package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot implements PIDOutput {

    Hand leftHand = GenericHID.Hand.kLeft;
    Hand rightHand = GenericHID.Hand.kRight;
    XboxController controller = new XboxController(0);
    XboxController secondary = controller;
    //XboxController secondary = new XboxController(1);

    Compressor compressor = new Compressor(61);

    AHRS ahrs = new AHRS(SPI.Port.kMXP);
    private static double kPTurn = 0.0125;
    private static double kITurn = 0.00;
    private static double kDTurn = 0.005;
    private static double kFTurn = 0.00;
    private static double kToleranceDegrees = 2.0f;

    private static double kP = 0.00;
    private static double kI = 0.00;
    private static double kD = 0.00;
    private static double kF = 0.00;

    private static boolean hadDoublePOV = false; // TODO: rename because a bit misleading
    private static boolean isFieldCentric = true;
    private static boolean armsUp = true;
    private static boolean intakeActive = false;

    WPI_TalonSRX frontRight = new WPI_TalonSRX(3);
    WPI_TalonSRX backRight = new WPI_TalonSRX(1);
    WPI_TalonSRX frontLeft = new WPI_TalonSRX(4);
    WPI_TalonSRX backLeft = new WPI_TalonSRX(2);
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

    public void robotInit() {
        elevator.setSelectedSensorPosition(0);

        frontLeft.setInverted(true);
        backRight.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(true);

        compressor.clearAllPCMStickyFaults();
        compressor.setClosedLoopControl(true);

        turnController.setInputRange(-180.0, 180.0);
        turnController.setOutputRange(-1, 1);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        turnController.enable();
        turnController.setSetpoint(0);
    }

    public void autonomousInit(){
        turnController.enable();
    }

    public void autonomousPeriodic(){

    }

    public void teleopInit(){
        turnController.enable();
        myDrive.setSafetyEnabled(true);
    }

    public void teleopPeriodic() {
        while(isOperatorControl()) {
            hatchMech();
            //reset(); Resets Gyro When you press A
            spinArms();
            adjustElevator();
            runIntake();
            centricToggle();

            if (!ahrs.isConnected()) {
                isFieldCentric = false;
            }

            if (controller.getTriggerAxis(leftHand) > 0.05 || controller.getTriggerAxis(rightHand) > 0.05) {
                strafe();
                snapToAngle();
            } else if (isFieldCentric) {
                driveCentric();
                snapToAngle();
            } else {
                driveStandard();
            }

            SmartDashboard.putNumber("angle", ahrs.getAngle());
            SmartDashboard.putNumber(   "IMU_Yaw",               ahrs.getYaw());
            SmartDashboard.putNumber(   "IMU_FusedHeading",      ahrs.getFusedHeading());
            SmartDashboard.putNumber(   "IMU_TotalYaw",          ahrs.getAngle());
            SmartDashboard.putNumber(   "IMU_YawRateDPS",        ahrs.getRate());
            SmartDashboard.putNumber(   "IMU_Accel_X",           ahrs.getWorldLinearAccelX());
            SmartDashboard.putNumber(   "IMU_Accel_Y",           ahrs.getWorldLinearAccelY());
            SmartDashboard.putBoolean(  "IMU_IsMoving",          ahrs.isMoving());
            SmartDashboard.putBoolean(  "IMU_IsRotating",        ahrs.isRotating());
            SmartDashboard.putNumber(   "Velocity_X",            ahrs.getVelocityX());
            SmartDashboard.putNumber(   "Velocity_Y",            ahrs.getVelocityY());
            SmartDashboard.putNumber(   "Displacement_X",        ahrs.getDisplacementX());
            SmartDashboard.putNumber(   "Displacement_Y",        ahrs.getDisplacementY());

            /*kP = SmartDashboard.getNumber("kP", .00);
            kI = SmartDashboard.getNumber("kI", .00);
            kD = SmartDashboard.getNumber("kD", .00);
            kF = SmartDashboard.getNumber("kF", .00);*/

            //turnController.setPID(kP, kI, kD, kF);
            SmartDashboard.putNumber("kP", turnController.getP());
            SmartDashboard.putNumber("kI", turnController.getI());
            SmartDashboard.putNumber("kD", turnController.getD());
            SmartDashboard.putNumber("kF", turnController.getF());

            SmartDashboard.putBoolean("isCentric", isFieldCentric);

            SmartDashboard.putNumber(   "Target",            turnController.getSetpoint());
            SmartDashboard.putNumber(   "Set Point",            turnController.getSetpoint());
            SmartDashboard.putBoolean(   "onTarget",            turnController.onTarget());


            SmartDashboard.putNumber(   "Xbox X",            controller.getX(leftHand));
            SmartDashboard.putNumber(   "Xbox Y",            controller.getY(leftHand));
            SmartDashboard.putNumber(   "Rotate to Angle Rate",        rotateToAngleRate);
            SmartDashboard.putNumber(   "Get Angle",               ahrs.getAngle());

            SmartDashboard.putBoolean("NavX Connected", ahrs.isConnected());

            SmartDashboard.putBoolean("Arms Down", armsUp);
            Timer.delay(.005);
        }
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
        if (controller.getRawButton(9)) {
            ahrs.reset();
        }
    }

    public void centricToggle() {
        if (controller.getRawButtonReleased(5)) {
            isFieldCentric = !isFieldCentric;
        } else {
            return;
        }
    }

    public void snapToAngle(){
        if (controller.getPOV() == -1) { // no controller POV pressed
            hadDoublePOV = false;
            return;
        }

        int controllerPOV = controller.getPOV();

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

    public void strafe(){
        if(controller.getTriggerAxis(leftHand) > 0.05 && controller.getTriggerAxis(rightHand) > 0.05) {
            return;
        } else if (controller.getTriggerAxis(leftHand) > 0.05) {
            frontLeft.set(controller.getTriggerAxis(leftHand));
            backLeft.set(-controller.getTriggerAxis(leftHand));
            frontRight.set(controller.getTriggerAxis(leftHand));
            backRight.set(-controller.getTriggerAxis(leftHand));
        } else if (controller.getTriggerAxis(rightHand) > 0.05) {
            frontLeft.set(-controller.getTriggerAxis(rightHand));
            backLeft.set(controller.getTriggerAxis(rightHand));
            frontRight.set(-controller.getTriggerAxis(rightHand));
            backRight.set(controller.getTriggerAxis(rightHand));
        } else {
            return;
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

    public void driveCentric(){
        myDrive.driveCartesian(-controller.getX(leftHand), controller.getY(leftHand), rotateToAngleRate, ahrs.getAngle() - 2*turnController.getSetpoint());
    }

    private void adjustElevator(){
        SmartDashboard.putNumber("elevator Encoder", elevator.getSelectedSensorPosition());

        if(secondary.getRawButton(4)) {
            lifter.setTarget(11300);
        } else if(secondary.getRawButton(3)) {
            lifter.setTarget(0);
        } else if (secondary.getRawButton(3) && secondary.getRawButton(4)) {
            lifter.setTarget(5500);
        }  else {
            return;
        }

        lifter.run();
    }

    private void runIntake() {
        if (secondary.getBButtonReleased()) {
            intakeActive = !intakeActive;
        }

        if (intakeActive) {
            intake.set(.25);
        } else if (secondary.getRawButton(6)) {
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
            hatchMechanism.set(DoubleSolenoid.Value.kReverse);
        } else {
            hatchMechanism.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void driveStandard() {
        myDrive.driveCartesian(-controller.getX(leftHand), controller.getY(leftHand), controller.getX(rightHand));
    }
}