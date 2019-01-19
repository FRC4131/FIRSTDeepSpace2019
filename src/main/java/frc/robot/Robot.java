package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;


public class Robot extends TimedRobot implements PIDOutput {
	Hand LeftHand = GenericHID.Hand.kLeft;
	Hand RightHand = GenericHID.Hand.kRight;
	XboxController Controller = new XboxController(0);
	AHRS ahrs = new AHRS(SPI.Port.kMXP);

	PIDController turnController;
	final double kPTurn = 0.0145;
	final double kITurn = 0.00;
	final double kDTurn = 0.03;
	final double kToleranceDegrees = 2.0f;

    PIDController driveController;
    final double kPDrive = 0.00;
    final double kIDrive = 0.00;
    final double kDDrive = 0.00;
    final double kToleranceDistance = 5;

    private boolean hadDoublePOV = false; // TODO: rename because a bit misleading
    private boolean isFieldCentric = true;

    final static double kCollisionThreshold_DeltaG = 2f;
    double last_world_linear_accel_x;
    double last_world_linear_accel_y;

	// Talons:
	Talon FrontRight = new Talon(3);
	Talon BackRight = new Talon(2);
	Talon FrontLeft = new Talon(1);
	Talon BackLeft = new Talon(0);

	double rotateToAngleRate;
    double driveToRate;
    DrivePID drivePID = new DrivePID();
	// Drive Base
	MecanumDrive myDrive = new MecanumDrive(BackRight, FrontRight, BackLeft, FrontLeft);

    private static final Map<Integer, Integer> angleMapping;
    static {
        angleMapping = new HashMap<>();
        angleMapping.put(45, 30);
        angleMapping.put(135, 150);
        angleMapping.put(225, 210);
        angleMapping.put(315, 330);
    }

	public void robotInit() {
        turnController = new PIDController(kPTurn, kITurn, kDTurn, ahrs, this);
        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-1, 1);
        turnController.setContinuous(true);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        driveController = new PIDController(kPDrive, kIDrive, kDDrive, ahrs, drivePID);//TODO:fix the input source;
        driveController.setOutputRange(-1,1);
        driveController.setContinuous(false);
        driveController.setAbsoluteTolerance(kToleranceDistance);
        myDrive.setDeadband(.1);
    }

    public void autonomousInit(){
	    turnController.enable();
    }

    public void autonomousPeriodic(){

    }

    public void teleopInit(){
        ahrs.reset();
        turnController.setSetpoint(0);
	    turnController.enable();
	}

	public void teleopPeriodic() {
        Reset();//Resets Gyro When you press A

        centricToggle();

        collisionDetection();

        //if (!ahrs.isConnected()) {
          //  isFieldCentric = false;
        //}

        if (Controller.getTriggerAxis(LeftHand) > 0.05 || Controller.getTriggerAxis(RightHand) > 0.05) {
            strafe();
            SnapToAngle();
        } else if (isFieldCentric) {
            if (Math.sqrt(Math.pow(Controller.getX(RightHand), 2) + Math.pow(Controller.getY(RightHand), 2)) > 0.2) {
                targetAngleCorrection();
            }
            driveCentric();
            SnapToAngle();
        } else {
            driveStandard();
        }

        SmartDashboard.putNumber("X Displacement", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Y Displacement", ahrs.getDisplacementY());
        SmartDashboard.putNumber("Target", turnController.getSetpoint());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("POV", Controller.getPOV());
        SmartDashboard.putBoolean("hadDoublePOV", hadDoublePOV);
        SmartDashboard.putBoolean("Field Centric Enabled", isFieldCentric);
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
	    controllerPOV = angleMapping.getOrDefault(controllerPOV, controllerPOV); // re-map angle to be correct
	    controllerPOV = (controllerPOV > 180) ? controllerPOV - 360 : controllerPOV; // adjust range to (-180, 180]

        if (controllerPOV % 90 != 0) { // Controller POV is at mixed angle
            hadDoublePOV = true;
            turnController.setSetpoint(controllerPOV);
        } else if (!hadDoublePOV) { // Controller POV only has one pressed AND no double POV pressed yet
            turnController.setSetpoint(controllerPOV);
        }
    }

    public void targetAngleCorrection() {
	    double x = Controller.getX(RightHand);
	    double y = Controller.getY(RightHand);

	    double angle = Math.atan(y/x) * 180 / Math.PI;
	    if (x < 0) {
	        angle += 180;
        }

        double diff = 270 - angle;

	    while (diff > 180) diff -= 360;
	    while (diff <= -180) diff += 360;

	    double correction = diff / 400; // TODO: magic number

        double newPoint = turnController.getSetpoint() - correction;
        if (newPoint > 180) newPoint -= 360;
        if (newPoint <= -180) newPoint += 360;

	    turnController.setSetpoint(newPoint);
    }

    public void strafe(){
	    double left = Controller.getTriggerAxis(LeftHand);
	    double right = Controller.getTriggerAxis(RightHand);

	    double power = right - left;
        if (Math.abs(power) > 0.05 && isFieldCentric) {
            myDrive.driveCartesian(power, 0,
                    rotateToAngleRate, ahrs.getYaw() -  turnController.getSetpoint());
//            myDrive.driveCartesian(power, 0, rotateToAngleRate, 0);
        } else if (Math.abs(power) > 0.05) {
            myDrive.driveCartesian(power, 0, 0);
        }
    }

    public void driveCentric(){
        myDrive.driveCartesian(Controller.getX(LeftHand), -Controller.getY(LeftHand),
                rotateToAngleRate, ahrs.getYaw()- 2 * turnController.getSetpoint());
    }

    public void driveStandard() {
	    myDrive.driveCartesian(Controller.getX(LeftHand), -Controller.getY(LeftHand), Controller.getX(RightHand));
	}

	public void collisionDetection() {
        boolean collisionDetected = false;

        double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;

        if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG) ||
                (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
            collisionDetected = true;
        }
        SmartDashboard.putBoolean("CollisionDetected", collisionDetected);
        if (collisionDetected) {
            Controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
            Controller.setRumble(GenericHID.RumbleType.kRightRumble, 1);
        } else {
            Controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            Controller.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }

    }

	private class DrivePID implements PIDOutput {
        @Override
        public void pidWrite(double output) {
            driveToRate = output;
        }
    }
}