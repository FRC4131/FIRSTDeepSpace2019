package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot implements PIDOutput {
    Hand LeftHand = GenericHID.Hand.kLeft;
    Hand RightHand = GenericHID.Hand.kRight;
    XboxController Controller = new XboxController(0);
    AHRS ahrs = new AHRS(SPI.Port.kMXP);
    final double kPTurn = 0.0125;
    final double kITurn = 0.00;
    final double kDTurn = 0.005;
    final double kFTurn = 0.00;
    final double kToleranceDegrees = 2.0f;
    static double kP = 0.00;
    static double kI = 0.00;
    static double kD = 0.00;
    static double kF = 0.00;
    private boolean armsToggle;
    private boolean hadDoublePOV = false; // TODO: rename because a bit misleading
    private boolean isFieldCentric = true;
    // Talons:
    WPI_TalonSRX FrontRight = new WPI_TalonSRX(3);
    WPI_TalonSRX BackRight = new WPI_TalonSRX(1);
    WPI_TalonSRX FrontLeft = new WPI_TalonSRX(4);
    WPI_TalonSRX BackLeft = new WPI_TalonSRX(2);
    WPI_TalonSRX LeftArm = new WPI_TalonSRX(5);
    WPI_TalonSRX RightArm = new WPI_TalonSRX(6);
    double rotateToAngleRate;
    // Drive Base
    MecanumDrive myDrive = new MecanumDrive(FrontLeft,BackLeft,FrontRight,BackRight);
    PIDController turnController = new PIDController(kPTurn, kITurn, kDTurn,kFTurn, ahrs, this);
    public void robotInit() {

        FrontLeft.setInverted(true);
        BackRight.setInverted(true);
        FrontRight.setInverted(true);
        BackLeft.setInverted(true);


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

            Reset();//Resets Gyro When you press A
            checkSpinArms();
            centricToggle();
            if (!ahrs.isConnected()) {
                isFieldCentric = false;
            }

            if (Controller.getTriggerAxis(LeftHand) > 0.05 || Controller.getTriggerAxis(RightHand) > 0.05) {
                strafe();
                SnapToAngle();
            } else if (isFieldCentric) {
                driveCentric();
                SnapToAngle();
            } else {
//                driveStandard();
            }
            SmartDashboard.putNumber("angle", ahrs.getAngle());
            /* Code instrumentation */
//            SmartDashboard.putNumber(   "PDP Chan 0",            pdp.getCurrent(0));
//            SmartDashboard.putNumber(   "PDP Chan 1",            pdp.getCurrent(1));
//            SmartDashboard.putNumber(   "PDP Chan 2",            pdp.getCurrent(2));
//            SmartDashboard.putNumber(   "PDP Chan 3",            pdp.getCurrent(3));
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

            kP = SmartDashboard.getNumber("kP", .00);
            kI = SmartDashboard.getNumber("kI", .00);
            kD = SmartDashboard.getNumber("kD", .00);
            kF = SmartDashboard.getNumber("kF", .00);

            turnController.setPID(kP, kI, kD, kF);
            SmartDashboard.putNumber("kP", turnController.getP());
            SmartDashboard.putNumber("kI", turnController.getI());
            SmartDashboard.putNumber("kD", turnController.getD());
            SmartDashboard.putNumber("kF", turnController.getF());

/*		if ( Controller.getRawButton(1)) {
            ahrs.reset();
        }
*/

            SmartDashboard.putBoolean("isCentric", isFieldCentric);

            SmartDashboard.putNumber(   "Target",            turnController.getSetpoint());
            SmartDashboard.putNumber(   "Set Point",            turnController.getSetpoint());
            SmartDashboard.putBoolean(   "onTarget",            turnController.onTarget());


            SmartDashboard.putNumber(   "Xbox X",            Controller.getX(LeftHand));
            SmartDashboard.putNumber(   "Xbox Y",            Controller.getY(LeftHand));
            SmartDashboard.putNumber(   "Rotate to Angle Rate",        rotateToAngleRate);
            SmartDashboard.putNumber(   "Get Angle",               ahrs.getAngle());
            Timer.delay(.005);
        }
    }

    @Override
    public void pidWrite(double output) {
        if (turnController.onTarget()) { // deadband
            rotateToAngleRate = 0;
        } else {
            rotateToAngleRate = output;
        }
    }

    public void Reset(){
        if (Controller.getRawButton(1)) {
            ahrs.reset();
        }
    }

    public void centricToggle() {
        if (Controller.getStartButton()) {
            isFieldCentric = !isFieldCentric;
        } else {
            return;
        }
    }

    public void SnapToAngle(){
        if (Controller.getPOV() == -1) { // no Controller POV pressed
            hadDoublePOV = false;
            return;
        }

        int controllerPOV = Controller.getPOV();
        if (controllerPOV > 180) {
            controllerPOV -= 360;
        }

        if (controllerPOV % 90 != 0) { // Controller POV is at mixed angle
            hadDoublePOV = true;
            turnController.setSetpoint(controllerPOV);
        } else if (!hadDoublePOV) { // Controller POV only has one pressed AND no double POV pressed yet
            turnController.setSetpoint(controllerPOV);
        }
    }

    public void strafe(){
        if(Controller.getTriggerAxis(LeftHand) > 0.05 && Controller.getTriggerAxis(RightHand) > 0.05) {
            return;
        } else if (Controller.getTriggerAxis(LeftHand) > 0.05) {
            FrontLeft.set(Controller.getTriggerAxis(LeftHand));
            BackLeft.set(-Controller.getTriggerAxis(LeftHand));
            FrontRight.set(Controller.getTriggerAxis(LeftHand));
            BackRight.set(-Controller.getTriggerAxis(LeftHand));
        } else if (Controller.getTriggerAxis(RightHand) > 0.05) {
            FrontLeft.set(-Controller.getTriggerAxis(RightHand));
            BackLeft.set(Controller.getTriggerAxis(RightHand));
            FrontRight.set(-Controller.getTriggerAxis(RightHand));
            BackRight.set(Controller.getTriggerAxis(RightHand));
        } else {
            return;
        }
    }

    public void checkSpinArms(){

        if(Controller.getRawButton(2)){
            armsToggle = true;
        } else {
            armsToggle = false;
        }
        if(armsToggle){
            LeftArm.set(-.75);
            RightArm.set(.75);
        } else {
            LeftArm.set(0);
            RightArm.set(0);
        }
    }

    public void driveCentric(){
        myDrive.driveCartesian(-Controller.getX(LeftHand), Controller.getY(LeftHand), rotateToAngleRate, ahrs.getAngle() - 2*turnController.getSetpoint());
    }

//    public void driveStandard() {
//        myDrive.driveCartesian(Controller.getX(LeftHand), Controller.getY(LeftHand), Controller.getX(RightHand));
//    }

}