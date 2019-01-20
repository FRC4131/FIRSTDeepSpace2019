package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.LinkedList;
import java.util.stream.Stream;


public class AutoPID extends TimedRobot implements PIDOutput {
	Hand LeftHand = Hand.kLeft;
	Hand RightHand = Hand.kRight;
	XboxController Controller = new XboxController(0);
	AHRS ahrs = new AHRS(SPI.Port.kMXP);

	PIDController turnController;
	final double kPTurn = 0.00;
	final double kITurn = 0.00;
	final double kDTurn = 0.00;
	final double kToleranceDegrees = 2.0f;

	private LinkedList<double[]> PIDAdjustment = new LinkedList<>();

    private boolean hadDoublePOV = false; // TODO: rename because a bit misleading
    private boolean isFieldCentric = true;

	// Talons:
	WPI_TalonSRX FrontRight = new WPI_TalonSRX(3);
	WPI_TalonSRX BackRight = new WPI_TalonSRX(2);
	WPI_TalonSRX FrontLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX BackLeft = new WPI_TalonSRX(0);

	double rotateToAngleRate;

	// Drive Base
	MecanumDrive myDrive = new MecanumDrive(BackRight, FrontRight, BackLeft, FrontLeft);

	public void robotInit() {
        turnController = new PIDController(kPTurn, kITurn, kDTurn, ahrs, this);
        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-1, 1);
        turnController.setContinuous(true);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        myDrive.setDeadband(.1);
    }

    public void autonomousInit(){
	    turnController.enable();
    }

    public void autonomousPeriodic(){

    }

    public void teleopInit(){
	    PIDAdjustment.add(new double[] {kPTurn, kITurn, kDTurn, 0});
        ahrs.reset();
        turnController.setSetpoint(0);
	    turnController.enable();
	}

	public void teleopPeriodic() {
        Reset();//Resets Gyro When you press A

        centricToggle();


        if (isFieldCentric) {
//            driveCentric();
            SnapToAngle();
        } else {
            driveStandard();
        }

        SmartDashboard.putBoolean("ahrs connected", ahrs.isConnected());
        SmartDashboard.putNumber("X Displacement", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Y Displacement", ahrs.getDisplacementY());
        SmartDashboard.putNumber("Target", turnController.getSetpoint());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("POV", Controller.getPOV());
        SmartDashboard.putBoolean("hadDoublePOV", hadDoublePOV);
        SmartDashboard.putBoolean("Field Centric Enabled", isFieldCentric);

        SmartDashboard.putNumberArray("best PID", getBestPID());
	}

	private void makeTurn(double targetAngle) {
	    double startTime = Timer.getFPGATimestamp();

	    turnController.setSetpoint(targetAngle);

	    while (!isTurnDone(startTime)) {
	        driveCentric();
        }

        double endAngle = ahrs.getAngle();
        double endTime = Timer.getFPGATimestamp();

        double angleDiff = Math.abs(endAngle - targetAngle);
        double timeDiff = endTime - startTime;

        double error = 2 * angleDiff + 5 * timeDiff;

        PIDAdjustment.getLast()[3] = error;

        double[] nextPID;
        if (PIDAdjustment.size() == 1) {
            nextPID = new double[] {0.1, 0.1, 0.1, 0};
        } else {
            nextPID = getNextPID();
        }
        PIDAdjustment.add(nextPID);

        setPID(nextPID);
    }

    private boolean isTurnDone(double start) {
	    if (Timer.getFPGATimestamp() > start + 5) {
	        return true;
        }

	    double velocity = Math.sqrt(
	            Math.pow(ahrs.getVelocityX(), 2) + Math.pow(ahrs.getVelocityY(), 2) + Math.pow(ahrs.getVelocityZ(), 2)
        );

        final double rateThreshold = 0.5; // degrees / second
        final double velocityThreshold = 0.1; // meters / second

        return Math.abs(ahrs.getRate()) < rateThreshold && velocity < velocityThreshold;
	}

    private double[] getNextPID() {
        Stream<double[]> bestPIDs = PIDAdjustment.stream().sorted((o1, o2) -> (int) ((o1[3] - o2[3]) * 10000));

        // assume at least 2 present
        double[] a = bestPIDs.findFirst().get();
        double[] b = bestPIDs.skip(1).findFirst().get();

        double picker = Math.random();
        double p = (picker < 0.33) ? (a[0] + b[0]) / 2 : (Math.random() < 0.5) ? a[0] : b[0];
        double i = (picker < 0.66 && picker > 0.33) ? (a[1] + b[1]) / 2 : (Math.random() < 0.5) ? a[1] : b[1];
        double d = (picker > 0.66) ? (a[2] + b[2]) / 2 : (Math.random() < 0.5) ? a[2] : b[2];

        p += (Math.random() - 0.5) / 10000;
        i += (Math.random() - 0.5) / 10000;
        d += (Math.random() - 0.5) / 10000;

        return new double[] {p, i, d, 0};
    }

    private double[] getBestPID() {
        Stream<double[]> bestPIDs = PIDAdjustment.stream().sorted((o1, o2) -> (int) ((o1[3] - o2[3]) * 10000));
        double[] best = bestPIDs.findFirst().get();
        return new double[] {best[0], best[1], best[2]};
    }

    private void setPID(double[] pid) {
	    turnController.setPID(pid[0], pid[1], pid[2]);
    }

	@Override
	public void pidWrite(double output) {
	    rotateToAngleRate = output;
	}

	public void Reset() {
        if (Controller.getRawButton(1)) {
            ahrs.reset();
            turnController.setSetpoint(0);
        }
    }

    public void centricToggle() {
	    if (Controller.getStartButtonPressed()) {
	        isFieldCentric = !isFieldCentric;
        }
    }

    public void SnapToAngle(){
	    if (Controller.getPOV() == -1) { // no Controller POV pressed
	        hadDoublePOV = false;
	        return;
        }

        double before = turnController.getSetpoint();

        int controllerPOV = Controller.getPOV();
        if (controllerPOV > 180) {
            controllerPOV -= 360;
        }

        if (controllerPOV == before) {
            return; // do nothing if setpoint does not change
        }

        if (controllerPOV % 90 != 0) { // Controller POV is at mixed angle
            hadDoublePOV = true;
            makeTurn(controllerPOV);
        } else if (!hadDoublePOV) { // Controller POV only has one pressed AND no double POV pressed yet
            makeTurn(controllerPOV);
        }

    }

    public void driveCentric(){
        myDrive.driveCartesian(Controller.getX(LeftHand), -Controller.getY(LeftHand),
                rotateToAngleRate, ahrs.getYaw()- 2 * turnController.getSetpoint());
    }

    public void driveStandard() {
	    myDrive.driveCartesian(Controller.getX(LeftHand), -Controller.getY(LeftHand), Controller.getX(RightHand));
	}
}