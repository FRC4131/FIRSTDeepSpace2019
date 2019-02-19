package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.crypto.Mac;
import java.util.HashMap;
import java.util.Map;

public class Robot extends TimedRobot implements PIDOutput {

    Hand leftHand = GenericHID.Hand.kLeft;
    Hand rightHand = GenericHID.Hand.kRight;
    XboxController controller = new XboxController(0);
    XboxController secondary = new XboxController(1);

    Compressor compressor = new Compressor(61);

    AHRS ahrs = new AHRS(SPI.Port.kMXP);
    private static double kPTurn = 0.0105;
    private static double kITurn = 0.00;
    private static double kDTurn = 0.005;
    private static double kFTurn = 0.00;
    private static double kToleranceDegrees = 2.0f;

    private static boolean hadDoublePOV = false; // TODO: rename because a bit misleading
    private static boolean isFieldCentric = true;
    private static boolean armsUp = true;
    private static boolean intakeActive = false;
    private static boolean isHatchDown = false;

    //the centers of the two retro-reflective targets, if both the x and y of a target are zero it isn't detected
    private static double ballZeroCenterX, ballZeroCenterY, ballOneCenterX, ballOneCenterY, hatchZeroCenterX, hatchZeroCenterY, hatchOneCenterX, hatchOneCenterY;
    private static double ballContours, hatchContours;

    private static Map<Integer, Integer> angleMapping = new HashMap<>();

    static {
        angleMapping.put(45, 30);
        angleMapping.put(135, 150);
        angleMapping.put(225, 240);
        angleMapping.put(315, 330);
    }

    WPI_TalonSRX frontRight = new WPI_TalonSRX(2);
    WPI_TalonSRX backRight = new WPI_TalonSRX(4);
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(1);
    WPI_TalonSRX backLeft = new WPI_TalonSRX(3);
    WPI_TalonSRX leftArm = new WPI_TalonSRX(5);
    WPI_TalonSRX rightArm = new WPI_TalonSRX(6);
    WPI_TalonSRX elevator = new WPI_TalonSRX(7);
    WPI_TalonSRX intake = new WPI_TalonSRX(8);

    Lifter lifter = new Lifter(elevator);

    DoubleSolenoid hatchDeploy = new DoubleSolenoid(61, 2,3);
    DoubleSolenoid armsDeploy = new DoubleSolenoid(61, 4,5);
    DoubleSolenoid hatchMechanism = new DoubleSolenoid(61,0,1);

    private static double rotateToAngleRate;

    MecanumDrive myDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    PIDController turnController = new PIDController(kPTurn, kITurn, kDTurn,kFTurn, ahrs, this);

    AutoStrafer autoStrafer = new AutoStrafer();
    PIDController strafeController = new PIDController(1.2, 0, 0.6, 0, autoStrafer, autoStrafer);

    public void robotInit() {
        elevator.setSelectedSensorPosition(0);

        frontLeft.setInverted(true);
        backRight.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(true);

        compressor.clearAllPCMStickyFaults();
        compressor.setClosedLoopControl(true);
        myDrive.setDeadband(.1);
        turnController.setInputRange(-180.0, 180.0);
        turnController.setOutputRange(-1, 1);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        turnController.enable();
        turnController.setSetpoint(0);

        //strafeController.setInputRange(-1000, 1000);
        strafeController.setOutputRange(-0.5, 0.5);
        strafeController.setAbsoluteTolerance(5);
        strafeController.setContinuous(false);
        strafeController.enable();
        strafeController.setSetpoint(0);
    }

    public void autonomousInit(){
        turnController.enable();
        myDrive.setSafetyEnabled(true);
        isHatchDown = true;
    }

    public void autonomousPeriodic(){
        while (isAutonomous()) {
            ntReader();
            run();
            smartDash();
            Timer.delay(.005);
        }
    }

    public void teleopInit(){
        turnController.enable();
        myDrive.setSafetyEnabled(true);
        isHatchDown = true;
    }

    public void teleopPeriodic() {
        while(isOperatorControl()) { // avoid extra teleop baggage
            ntReader();
            run();
            smartDash();
            Timer.delay(.005); // a bit odd, but told to not remove
        }
    }

    public void run() {
        hatchMech();
        deployHatchMech();
        reset();
        spinArms();
        adjustElevator();
        lifter.run();
        runIntake();
        centricToggle();

        if (controller.getRawButton(1)) {
            autoStrafer.run();
            return;
        }

        if (!ahrs.isConnected()) {
            isFieldCentric = false;
        }

        if (isFieldCentric) {
            driveCentric();
            snapToAngle();
        } else {
            driveStandard();
        }
    }

    private void ntReader() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("vision");
        NetworkTableEntry ballZeroX = table.getEntry("ballZeroX");
        NetworkTableEntry ballZeroY = table.getEntry("ballZeroY");
        NetworkTableEntry ballOneX = table.getEntry("ballOneX");
        NetworkTableEntry ballOneY = table.getEntry("ballOneY");
        NetworkTableEntry ballContours = table.getEntry("ballContoursCount");
        NetworkTableEntry hatchZeroX = table.getEntry("hatchZeroX");
        NetworkTableEntry hatchZeroY = table.getEntry("hatchZeroY");
        NetworkTableEntry hatchOneX = table.getEntry("hatchOneX");
        NetworkTableEntry hatchOneY = table.getEntry("hatchOneY");
        NetworkTableEntry hatchContours = table.getEntry("hatchContoursCount");
        inst.startServer();
        inst.setServerTeam(4131);
        this.ballZeroCenterX = ballZeroX.getDouble(0);
        this.ballZeroCenterY = ballZeroY.getDouble(0);
        this.ballOneCenterX = ballOneX.getDouble(0);
        this.ballOneCenterY = ballOneY.getDouble(0);
        this.ballContours = ballContours.getDouble(0);
        this.hatchZeroCenterX = hatchZeroX.getDouble(0);
        this.hatchZeroCenterY = hatchZeroY.getDouble(0);
        this.hatchOneCenterX = hatchOneX.getDouble(0);
        this.hatchOneCenterY = hatchOneY.getDouble(0);
        this.hatchContours = hatchContours.getDouble(0);
    }

    public void smartDash() {
        SmartDashboard.putBoolean("isCentric", isFieldCentric);

        SmartDashboard.putNumber("Target", turnController.getSetpoint());
        SmartDashboard.putNumber("Set Point", turnController.getSetpoint());
        SmartDashboard.putBoolean("onTarget", turnController.onTarget());
        SmartDashboard.putNumber("Rotate to Angle Rate", rotateToAngleRate);
        SmartDashboard.putNumber("Get Angle", ahrs.getAngle());

        SmartDashboard.putBoolean("NavX Connected", ahrs.isConnected());

        SmartDashboard.putBoolean("Arms Up", armsUp);

        SmartDashboard.putNumber("ballZeroX", ballZeroCenterX);
        SmartDashboard.putNumber("ballZeroY", ballZeroCenterY);
        SmartDashboard.putNumber("ballOneX", ballOneCenterX);
        SmartDashboard.putNumber("ballOneY", ballOneCenterY);
        SmartDashboard.putNumber("ballContours", ballContours);
        SmartDashboard.putNumber("hatchZeroX", hatchZeroCenterX);
        SmartDashboard.putNumber("hatchZeroY", hatchZeroCenterY);
        SmartDashboard.putNumber("hatchOneX", hatchOneCenterX);
        SmartDashboard.putNumber("hatchOneY", hatchOneCenterY);
        SmartDashboard.putNumber("hatchContours", hatchContours);
    }

    @Override
    public void pidWrite(double output) {
        if (turnController.onTarget()) {
            rotateToAngleRate = 0;
        } else {
            rotateToAngleRate = output;
        }
    }

    public void reset(){
        if (controller.getRawButton(3)) {
            ahrs.reset();
        }
    }

    public void centricToggle() {
        if (controller.getRawButtonReleased(5)) {
            isFieldCentric = !isFieldCentric;
        }
    }

    public void snapToAngle(){
        if (controller.getPOV() == -1) { // no controller POV pressed
            hadDoublePOV = false;
            return;
        }

        int controllerPOV = controller.getPOV();
        controllerPOV = angleMapping.getOrDefault(controllerPOV, controllerPOV);

        if (controllerPOV > 180) {
            controllerPOV -= 360;
        }

        if (controllerPOV % 90 != 0) { // controller POV is at mixed angle
            hadDoublePOV = true;
            turnController.setSetpoint(controllerPOV);
        } else if (!hadDoublePOV) { // controller POV only has one pressed AND no double POV pressed yet
            turnController.setSetpoint(controllerPOV);
        }
    }

    public void deployHatchMech() {
        if (isHatchDown) {
            hatchDeploy.set(DoubleSolenoid.Value.kForward);
        } else {
            hatchDeploy.set(DoubleSolenoid.Value.kReverse);
        }
        if (secondary.getRawButtonReleased(5)) {
            isHatchDown = !isHatchDown;
        }
    }

    public void spinArms(){
        if(intakeActive){
            leftArm.set(-.35);
            rightArm.set(.35);
        } else {
            leftArm.set(0);
            rightArm.set(0);
        }
    }

    private void adjustElevator(){
        SmartDashboard.putNumber("elevator Encoder", elevator.getSelectedSensorPosition());

        if(secondary.getRawButton(4)) {
            lifter.setTarget(11300);
        } else if(secondary.getRawButton(3)) {
            lifter.setTarget(0);
        }
    }

    private void runIntake() {
        if (secondary.getBButtonReleased()) {
            intakeActive = !intakeActive;
        }

        if (intakeActive) {
            intake.set(1);
        } else if (secondary.getTriggerAxis(leftHand) > 0.05 || secondary.getTriggerAxis(rightHand) > 0.05) {
            intake.set(-1);
        } else {
            intake.set(0);
        }

        if (secondary.getStartButtonReleased()) {
            armsUp = !armsUp;
        }

        if (!armsUp) {
             armsDeploy.set(DoubleSolenoid.Value.kForward);
         } else {
             armsDeploy.set(DoubleSolenoid.Value.kReverse);
         }
    }

    private void hatchMech(){
        if(secondary.getRawButton(1)){
            hatchMechanism.set(DoubleSolenoid.Value.kForward);
        } else {
            hatchMechanism.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void driveCentric(){
        myDrive.driveCartesian(-controller.getX(leftHand), controller.getY(leftHand), rotateToAngleRate, -ahrs.getAngle());
    }

    public void driveStandard() {
        myDrive.driveCartesian(controller.getX(leftHand), -controller.getY(leftHand), controller.getX(rightHand));
    }

    public void strafeCentric(double val) { //TODO: test
        myDrive.driveCartesian(-val, controller.getY(leftHand), rotateToAngleRate, 0);
    }

    private class AutoStrafer implements PIDSource, PIDOutput {
        PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
        double pidOut = 0;

        public void run() {
            strafeCentric(pidOut);
        }

        @Override
        public void pidWrite(double output) {
            pidOut = output;
            SmartDashboard.putNumber("strafe val", pidOut);
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            this.pidSourceType = pidSource;
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return pidSourceType;
        }

        @Override
        public double pidGet() {
            double max = Math.max(hatchZeroCenterX, hatchOneCenterX);
            double min = Math.min(hatchZeroCenterX, hatchOneCenterX);
            if (pidSourceType == PIDSourceType.kDisplacement) {
                // assumes 0 is center of frame
                return (184 - (max + min)/2) / (max - min);
            } else {
                // I don't really care about velocity for these... I think.
                // TODO: If it doesn't work, may need to implement kVelocity
                return 0;
            }
        }
    }
}