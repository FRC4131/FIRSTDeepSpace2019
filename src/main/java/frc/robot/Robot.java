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

import java.util.HashMap;
import java.util.Map;

public class Robot extends TimedRobot implements PIDOutput {

    Hand leftHand = GenericHID.Hand.kLeft;
    Hand rightHand = GenericHID.Hand.kRight;
    XboxController controller = new XboxController(0);
    XboxController secondary = new XboxController(1);

    Compressor compressor = new Compressor(61);

    AHRS ahrs = new AHRS(SPI.Port.kMXP);
    private static double kPTurn = 0.01;
    private static double kITurn = 0.00;
    private static double kDTurn = 0.005;
    private static double kFTurn = 0.00;
    private static double kToleranceDegrees = 2.0f;

    private static boolean hadDoublePOV = false; // TODO: rename because a bit misleading
    private static boolean isFieldCentric = true;
    private static boolean armsUp = true;
    private static boolean intakeActive = false;
    private static boolean isHatchDown = false;
    private static boolean ballSwitch;

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

    DigitalInput ballSwitchIn = new DigitalInput(0);

    DoubleSolenoid hatchDeploy = new DoubleSolenoid(61, 2,3);
    DoubleSolenoid armsDeploy = new DoubleSolenoid(61, 4,5);
    DoubleSolenoid hatchMechanism = new DoubleSolenoid(61,0,1);

    private static double rotateToAngleRate;

    MecanumDrive myDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    PIDController turnController = new PIDController(kPTurn, kITurn, kDTurn,kFTurn, ahrs, this);

    //TODO: send vision center to network tables
    //too small - left too big - right
    // put the center average value in here to align
    private static final double VISION_CENTER_X = 171;

    AutoStrafer autoStrafer = new AutoStrafer();
    PIDController strafeController = new PIDController(1, 0, 0.6, 0, autoStrafer, autoStrafer);

    AutoDriver autoDriver = new AutoDriver();
    PIDController driveController = new PIDController(0.003, 0, 0.01, 0, autoDriver, autoDriver);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private static final double INTAKE_CURRENT_LIMIT = 20;
    private static final int INTAKE_CURRENT_LIMIT_HARD = 30;

    public void robotInit() {
        elevator.setSelectedSensorPosition(0);

        intake.configPeakCurrentLimit(0);
        intake.configContinuousCurrentLimit(INTAKE_CURRENT_LIMIT_HARD);

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

        strafeController.setOutputRange(-0.5, 0.5);
//        strafeController.setAbsoluteTolerance(5);
        strafeController.setContinuous(false);
        strafeController.enable();
        strafeController.setSetpoint(0);

        driveController.setOutputRange(-0.4, 0.4);
        driveController.setAbsoluteTolerance(5);
        driveController.setContinuous(false);
        driveController.enable();
        driveController.setSetpoint(240); // pixel distance between targets when fully in (- a bit)

        inst.startServer();
        inst.setServerTeam(4131);
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
        snapToAngle();

        if (controller.getRawButton(1)) {
            //if (!(controller.getX(leftHand) < 0.05 || controller.getX(leftHand) > -0.05) || !(controller.getY(leftHand) < 0.05 || controller.getY(leftHand) > -0.05))  {
                autoStrafer.run(); // leave front-back ctrl
                autoStrafeDrive();
                return;
            //}
        }

        if (!ahrs.isConnected()) {
            isFieldCentric = false;
        }

        if (isFieldCentric) {
            driveCentric();
        } else {
            driveStandard();
        }
        ballSwitch = ballSwitchIn.get();
    }

    private void ntReader() {
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
        NetworkTableEntry debug = table.getEntry("debug");
        SmartDashboard.putNumber("debug", debug.getNumber(0).doubleValue());

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

        NetworkTableEntry dir = table.getEntry("strings");
        SmartDashboard.putString("dir", dir.getString("err"));
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
        SmartDashboard.putBoolean("Ball switch", ballSwitch);

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

        SmartDashboard.putBoolean("Strafe controller enabled", strafeController.isEnabled());
        SmartDashboard.putNumber("Strafe cont err", strafeController.getError());
        SmartDashboard.putNumber("autoStrafe val", autoStrafer.pidOut);

        SmartDashboard.putNumber("intake current", intake.getOutputCurrent());
        SmartDashboard.putNumber("pid for drive", autoDriver.pidOut);
        SmartDashboard.putNumber("center average", (hatchZeroCenterX + hatchOneCenterX) / 2);
        SmartDashboard.putNumber("center difference", hatchOneCenterX - hatchZeroCenterX);
        SmartDashboard.putNumber("Time", Timer.getMatchTime());
        SmartDashboard.putBoolean("Robot Connection", DriverStation.getInstance().isNewControlData());
        SmartDashboard.putBoolean("FMS Connection", DriverStation.getInstance().isFMSAttached());
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putBoolean("Brownout", RobotController.isBrownedOut());
        SmartDashboard.putBoolean("Compessor Enabled", compressor.enabled());
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
        if(intakeActive && ballSwitch){
            leftArm.set(-.3);
            rightArm.set(-.3);
        } else {
            leftArm.set(0);
            rightArm.set(0);
        }
    }

    private void adjustElevator(){
        SmartDashboard.putNumber("elevator Encoder", elevator.getSelectedSensorPosition());

        if(secondary.getRawButton(4)) {
            lifter.setTarget(-12500);
        } else if(secondary.getRawButton(3)) {
            lifter.setTarget(0);
        } else if(secondary.getBumper(rightHand)) {
            lifter.setTarget(-7200);
        }
    }

    private void runIntake() {
        if (secondary.getBButtonReleased()) {
            intakeActive = !intakeActive;
        }

        if (!ballSwitch) {
            intakeActive = false;
            armsUp = true;
        }

        if (intake.getOutputCurrent() > INTAKE_CURRENT_LIMIT) {
            intakeActive = false;
            armsUp = true;
        }

        if (intakeActive) {
            intake.set(0.5);
        } else if (secondary.getTriggerAxis(leftHand) > 0.05 || secondary.getTriggerAxis(rightHand) > 0.05) {
            intake.set(-0.75);
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
        if (secondary.getRawButtonPressed(1)) {
            hatchMechanism.set(DoubleSolenoid.Value.kForward);
        }
        if (secondary.getRawButtonReleased(1)) {
            hatchMechanism.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void driveCentric(){
        myDrive.driveCartesian(-controller.getX(leftHand), controller.getY(leftHand), rotateToAngleRate, -ahrs.getAngle());
    }

    public void driveStandard() {
        myDrive.driveCartesian(controller.getX(leftHand), -controller.getY(leftHand), controller.getX(rightHand));
    }

    public void strafeCentric(double x, double y) { //TODO: test
        myDrive.driveCartesian(-x, y, rotateToAngleRate, 0); // y as xSpeed is correct
    }

    public void autoStrafeDrive() {
        if (isStrafeAligned())
            strafeCentric(autoStrafer.pidOut, autoDriver.pidOut);
        else
            strafeCentric(autoStrafer.pidOut, 0);
    }

    private boolean isStrafeAligned() {
        double offCenter = Math.abs(VISION_CENTER_X - (hatchZeroCenterX + hatchOneCenterX)/2);
        return offCenter < 30;
    }

    private class AutoStrafer implements PIDSource, PIDOutput {
        PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
        double pidOut = 0;

        public void run() {
            strafeCentric(pidOut, controller.getY(leftHand));
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
            SmartDashboard.putNumber("vision debug", max - min);
            if (pidSourceType == PIDSourceType.kDisplacement) {
                // assumes 0 is center of frame
                return (VISION_CENTER_X - (max + min)/2) / (max - min);
            } else {
                // I don't really care about velocity for these... I think.
                // TODO: If it doesn't work, may need to implement kVelocity
                return 0;
            }
        }
    }

    private static class AutoDriver implements PIDSource, PIDOutput {
        PIDSourceType pidSourceType = PIDSourceType.kDisplacement;
        double pidOut = 0;

        @Override
        public void pidWrite(double output) {
            pidOut = -output;
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
                return max - min;
            } else {
                // I don't really care about velocity for these... I think.
                // TODO: If it doesn't work, may need to implement kVelocity
                return 0;
            }
        }
    }
}