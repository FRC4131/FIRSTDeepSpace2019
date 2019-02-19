package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;


public class AutoPID extends TimedRobot implements PIDOutput {

	Hand leftHand = Hand.kLeft;
	Hand rightHand = Hand.kRight;
	XboxController controller = new XboxController(0);
	AHRS ahrs = new AHRS(SPI.Port.kMXP);

	PIDController turnController;
	final double kPTurn = 0.00;
	final double kITurn = 0.00;
	final double kDTurn = 0.00;
	final double kToleranceDegrees = 2.0f;

	private Set<PIDSet> PIDAdjustment = new HashSet<>();

    private boolean hadDoublePOV = false; // TODO: rename because a bit misleading
    private boolean isFieldCentric = true;

	// Talons:
	WPI_TalonSRX frontRight = new WPI_TalonSRX(2);
	WPI_TalonSRX backRight = new WPI_TalonSRX(3);
	WPI_TalonSRX frontLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX backLeft = new WPI_TalonSRX(4);

	double rotateToAngleRate;

	// Drive Base
	MecanumDrive myDrive = new MecanumDrive(backRight, frontRight, backLeft, frontLeft);

	public void robotInit() {
        turnController = new PIDController(kPTurn, kITurn, kDTurn, ahrs, this);
        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-0.5, 0.5);
        turnController.setContinuous(true);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        myDrive.setDeadband(.1);
    }

    public void autonomousInit(){
	    turnController.enable();
    }

    public void autonomousPeriodic(){

    }

    private class PIDSet implements Comparable<PIDSet> {

	    private final double p;
        private final double i;
        private final double d;
        LinkedList<Double> errors = new LinkedList<>();

	    public PIDSet(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
        }

        public double getError() {
            double mean = errors.stream().mapToDouble(x -> x).average().orElse(0);
            double stdev = Math.sqrt(errors.stream().mapToDouble(x -> Math.pow(x - mean, 2)).sum() / (errors.size()));
            return (mean + stdev) * (1 + 0.01 * errors.size());
        }

        @Override
        public int compareTo(PIDSet o) {
	        return (int)(10_000 * (this.getError() - o.getError()));
        }

        public Set<PIDSet> recombine(PIDSet other) {
            double picker = Math.random();
            double p = (this.p + other.p) / 2;
            double i = (this.i + other.i) / 2;
            double d = (this.d + other.d) / 2;

            HashSet<PIDSet> out = new HashSet<>();
            out.add(new PIDSet(p, this.i, this.d));
            out.add(new PIDSet(p, other.i, other.d));
            //out.add(new PIDSet(this.p, i, this.d));
            //out.add(new PIDSet(this.p, this.i, d));

            return out;
        }

        public String toString() {
	        return String.format("PIDSet {P:%.3f, I:%.3f, D:%.3f, error:%.2f}", p, i, d, getError());
        }

        @Override
        public int hashCode() {
	        return (int) (1000*(p*i*d + p*i + p*d + i*d + p + i + d));
        }

        @Override
        public boolean equals(Object other) {
	        if (other instanceof PIDSet) {
	            PIDSet o = (PIDSet) other;
	            return p == o.p && i == o.i && d == o.d;
            } else {
	            return false;
            }
        }
    }

    public void teleopInit(){
	    PIDAdjustment.clear();
	    PIDSet a = new PIDSet(0.005, 0, 0);
        for (int i = 0; i < 5; i++) {
            ahrs.reset();
            turnController.setSetpoint(0);
            makeTurn(90, a);
        }

        PIDSet b = new PIDSet(0.08, 0.0, 0);
        for (int i = 0; i < 5; i++) {
            ahrs.reset();
            turnController.setSetpoint(0);
            makeTurn(90, b);
        }

        PIDAdjustment.add(a);
        PIDAdjustment.add(b);
        ahrs.reset();
        turnController.setSetpoint(0);
	    turnController.enable();
	}

	public void teleopPeriodic() {
        Reset();//Resets Gyro When you press A

        centricToggle();

        myDrive.stopMotor();

        if (isFieldCentric) {
            //driveCentric();
            SnapToAngle();
        } else {
            driveStandard();
        }

        SmartDashboard.putBoolean("ahrs connected", ahrs.isConnected());
        SmartDashboard.putNumber("X Displacement", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Y Displacement", ahrs.getDisplacementY());
        SmartDashboard.putNumber("Target", turnController.getSetpoint());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("POV", controller.getPOV());
        SmartDashboard.putBoolean("hadDoublePOV", hadDoublePOV);
        SmartDashboard.putBoolean("Field Centric Enabled", isFieldCentric);

        String text = "";
        for (PIDSet set : PIDAdjustment) {
            text += set + "\r\n";
        }
        SmartDashboard.putString("PIDsets", text);
        SmartDashboard.putString("bestPID", getBestPID().toString());
	}

	private void setupPIDSet() {
	    Set<PIDSet> sets = getNextPID();

	    for (PIDSet set : sets) {
            for (int i = 0; i < 5; i++) {
                ahrs.reset();
                turnController.setSetpoint(0);
                makeTurn(90, set);
            }

            PIDAdjustment.add(set);
        }
    }

	private void makeTurn(double targetAngle, PIDSet set) {
	    double startTime = Timer.getFPGATimestamp();

	    turnController.setPID(set.p, set.i, set.d);
	    turnController.setSetpoint(targetAngle);

	    while (!isTurnDone(startTime)) {
	        driveCentric();
        }

        double endAngle = ahrs.getAngle();
        double endTime = Timer.getFPGATimestamp();

        double angleDiff = Math.abs(endAngle - targetAngle);
        double timeDiff = endTime - startTime;

        double error = 2 * angleDiff + 5 * timeDiff;

        System.out.println(error);
        set.errors.add(error);
    }

    private boolean isTurnDone(double start) {
	    if (Timer.getFPGATimestamp() > start + 2.5) {
	        return true;
        }

	    if (Timer.getFPGATimestamp() < start + 0.2) {
	        return false;
        }

	    double velocity = Math.sqrt(Math.pow(ahrs.getVelocityX(), 2) + Math.pow(ahrs.getVelocityY(), 2) + Math.pow(ahrs.getVelocityZ(), 2));

        final double rateThreshold = 0.5; // degrees / second
        final double velocityThreshold = 0.1; // meters / second

        return Math.abs(ahrs.getRate()) < rateThreshold && velocity < velocityThreshold;
	}

    private Set<PIDSet> getNextPID() {
        Stream<PIDSet> bestPIDs = PIDAdjustment.stream().sorted();
        Iterator<PIDSet> best = bestPIDs.limit(2).iterator();
        //assume at least 2 present
        PIDSet a = best.next();
        PIDSet b = best.next();

        return a.recombine(b);
    }

    private PIDSet getBestPID() {
	    return PIDAdjustment.stream().sorted().findFirst().get();
    }

	@Override
	public void pidWrite(double output) {
	    rotateToAngleRate = output;
	}

	public void Reset() {
        if (controller.getRawButton(1)) {
            ahrs.reset();
            turnController.setSetpoint(0);
        }
    }

    public void centricToggle() {
	    if (controller.getStartButtonPressed()) {
	        isFieldCentric = !isFieldCentric;
        }
    }

    public void SnapToAngle(){
	    if (controller.getPOV() == -1) { // no controller POV pressed
	        hadDoublePOV = false;
	        return;
        }

	    if (controller.getPOV() == 90) {
	        setupPIDSet();
        } else if (controller.getPOV() == 180) {
	        PIDAdjustment = PIDAdjustment.stream().sorted().limit(4).collect(Collectors.toSet());
        } else if (controller.getPOV() == 270) {
//	        improveBestPID();
        }

        double before = turnController.getSetpoint();

        int controllerPOV = controller.getPOV();
        if (controllerPOV > 180) {
            controllerPOV -= 360;
        }

        if (controllerPOV == before) {
            return; // do nothing if setpoint does not change
        }

        if (controllerPOV % 90 != 0) { // controller POV is at mixed angle
            hadDoublePOV = true;
//            makeTurn(controllerPOV);
        } else if (!hadDoublePOV) { // controller POV only has one pressed AND no double POV pressed yet
//            makeTurn(controllerPOV);
        }

    }

    public void driveCentric(){
        myDrive.driveCartesian(controller.getX(leftHand), -controller.getY(leftHand),
                rotateToAngleRate, ahrs.getYaw()- 2 * turnController.getSetpoint());
    }

    public void driveStandard() {
	    myDrive.driveCartesian(controller.getX(leftHand), -controller.getY(leftHand), controller.getX(rightHand));
	}
}