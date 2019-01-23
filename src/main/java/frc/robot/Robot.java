package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
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
	final double kPTurn = 0.01;
	final double kITurn = 0.00;
	final double kDTurn = 0.05;
	final double kToleranceDegrees = 2.0f;
    private boolean hadDoublePOV = false; // TODO: rename because a bit misleading
    private boolean isFieldCentric = true;

    final static double kCollisionThreshold_DeltaG = 2f;
    double last_world_linear_accel_x;
    double last_world_linear_accel_y;

	// Talon SRXs:
    WPI_TalonSRX FrontRight = new WPI_TalonSRX(2);
    WPI_TalonSRX BackRight = new WPI_TalonSRX(3);
    WPI_TalonSRX FrontLeft = new WPI_TalonSRX(1);
    WPI_TalonSRX BackLeft = new WPI_TalonSRX(4);

	double rotateToAngleRate;
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
        ahrs.setPIDSourceType(PIDSourceType.kDisplacement);
        myDrive.setDeadband(.1);
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 500, 500, 10));
    }

    public void autonomousInit(){
	    ahrs.reset();
	    turnController.enable();
    }

    public void autonomousPeriodic(){
        myDrive.driveCartesian(0, 0, rotateToAngleRate);
        SmartDashboard.putNumber("X Displacement", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Y Displacement", ahrs.getDisplacementY());

    }

    public void teleopInit(){
        ahrs.reset();
        turnController.setSetpoint(0);
	    turnController.enable();
	    FrontRight.setNeutralMode(NeutralMode.Coast);
	    FrontLeft.setNeutralMode(NeutralMode.Coast);
	    BackRight.setNeutralMode(NeutralMode.Coast);
	    BackLeft.setNeutralMode(NeutralMode.Coast);
	}

	public void teleopPeriodic() {
        Reset();//Resets Gyro When you press A

        centricToggle();

        collisionDetection();

        if (!ahrs.isConnected()) {
          isFieldCentric = false;
        }

        if (Controller.getTriggerAxis(LeftHand) > 0.1 || Controller.getTriggerAxis(RightHand) > 0.1) {
            strafe();
            SnapToAngle();
        } else if (isFieldCentric) {
            if (Math.sqrt(Math.pow(Controller.getX(RightHand), 2) + Math.pow(Controller.getY(RightHand), 2)) > 0.8) {
               targetAngleCorrection();
            }
            driveCentric();
            SnapToAngle();
        } else {
            driveStandard();
        }

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

	    double power = left - right;
        if (Math.abs(power) > 0.05 && isFieldCentric) {
            myDrive.driveCartesian(power, 0,
                    rotateToAngleRate, ahrs.getYaw() -  turnController.getSetpoint());
            myDrive.driveCartesian(power, 0, rotateToAngleRate, 0);
        } else if (Math.abs(power) > 0.05) {
            myDrive.driveCartesian(power, 0, 0);
        }
    }

    public void driveCentric(){
        myDrive.driveCartesian(-Controller.getX(LeftHand), Controller.getY(LeftHand),
               rotateToAngleRate, ahrs.getYaw()- 2 * turnController.getSetpoint());
    }

    public void driveStandard() {
	    myDrive.driveCartesian(-Controller.getX(LeftHand), Controller.getY(LeftHand), Controller.getX(RightHand));
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

}