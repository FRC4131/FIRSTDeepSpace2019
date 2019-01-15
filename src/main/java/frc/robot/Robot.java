package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot implements PIDOutput {
	Hand LeftHand = GenericHID.Hand.kLeft;
	Hand RightHand = GenericHID.Hand.kRight;
	XboxController Controller = new XboxController(0);
	AHRS ahrs = new AHRS(SPI.Port.kMXP);
	static double kP = 0.0135;
	static double kI = 0.00;
	static double kD = 0.1;
	static double kF = 0.00;
	static double kToleranceDegrees = 2.0f;
	PIDController turnController;

	// Talons:
	Talon FrontRight = new Talon(3);
	Talon BackRight = new Talon(2);
	Talon FrontLeft = new Talon(1);
	Talon BackLeft = new Talon(0);
	double rotateToAngleRate;

	// Drive Base
	MecanumDrive myDrive = new MecanumDrive(BackRight, FrontRight, BackLeft, FrontLeft);

	public void robotInit() {
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this, .01);
        turnController.setInputRange(-180.0f, 180.0f);
        turnController.setOutputRange(-1, 1);
        myDrive.setDeadband(.1);
        turnController.setContinuous(true);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
    }

    public void teleopInit(){
		turnController.enable();
	}

	public void teleopPeriodic() {
        Reset();//Resets Gyro When you press A

        SnapToAngle();//Turn to angle

        if (Controller.getTriggerAxis(LeftHand) > 0.05 || Controller.getTriggerAxis(RightHand) > 0.05) {
            strafe();
        } else {
            DriveNormally();
        }

        SmartDashboard.putNumber("Target", turnController.getSetpoint());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
	}


	@Override
	public void pidWrite(double output) {
	    rotateToAngleRate = output;
	}

	public void Reset(){
        if ( Controller.getRawButton(1)) {
            ahrs.reset();
        }
    }

    public void SnapToAngle(){
        if(Controller.getPOV()== 0) turnController.setSetpoint(0); //0
        if(Controller.getPOV()== 45) turnController.setSetpoint(45);//90
        if(Controller.getPOV()== 90) turnController.setSetpoint(90);//180
        if(Controller.getPOV()== 135) turnController.setSetpoint(135);//-90
        if(Controller.getPOV()== 180) turnController.setSetpoint(180);//0
        if(Controller.getPOV()== 225) turnController.setSetpoint(-135);//90
        if(Controller.getPOV()== 270) turnController.setSetpoint(-90);//180
        if(Controller.getPOV()== 315) turnController.setSetpoint(-45);//-90

    }

    public void strafe(){
	    if(Controller.getTriggerAxis(LeftHand) > 0.05 && Controller.getTriggerAxis(RightHand) > 0.05) {
	        return;
        } else if (Controller.getTriggerAxis(LeftHand) > 0.05) {
	        FrontLeft.set(-Controller.getTriggerAxis(LeftHand));
	        BackLeft.set(Controller.getTriggerAxis(LeftHand));
	        FrontRight.set(-Controller.getTriggerAxis(LeftHand));
	        BackRight.set(Controller.getTriggerAxis(LeftHand));
        } else if (Controller.getTriggerAxis(RightHand) > 0.05) {
            FrontLeft.set(Controller.getTriggerAxis(RightHand));
            BackLeft.set(-Controller.getTriggerAxis(RightHand));
            FrontRight.set(Controller.getTriggerAxis(RightHand));
            BackRight.set(-Controller.getTriggerAxis(RightHand));
        } else {
	        return;
        }
	}

    public void DriveNormally(){
        myDrive.driveCartesian(Controller.getX(LeftHand), -Controller.getY(LeftHand),
                rotateToAngleRate, ahrs.getYaw()- 2 * turnController.getSetpoint());
    }
}