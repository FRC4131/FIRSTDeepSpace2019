package frc.robot;

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

	PIDController turnController;
	final double kPTurn = 0.0135;
	final double kITurn = 0.00;
	final double kDTurn = 0.1;
	final double kToleranceDegrees = 2.0f;

    PIDController driveController;
    final double kPDrive = 0.00;
    final double kIDrive = 0.00;
    final double kDDrive = 0.00;
    final double kToleranceDistance = 5;

    private boolean hadDoublePOV = false; // TODO: rename because a bit misleading
    private boolean isFieldCentric = true;

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
		turnController.enable();
	}

	public void teleopPeriodic() {
        Reset();//Resets Gyro When you press A

        centricToggle();

        if (!ahrs.isConnected()) {
            isFieldCentric = false;
        }

        if (Controller.getTriggerAxis(LeftHand) > 0.05 || Controller.getTriggerAxis(RightHand) > 0.05) {
            strafe();
            SnapToAngle();
        } else if (isFieldCentric){
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

	public void Reset(){
        if ( Controller.getRawButton(1)) {
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

    public void driveCentric(){
        myDrive.driveCartesian(Controller.getX(LeftHand), -Controller.getY(LeftHand),
                rotateToAngleRate, ahrs.getYaw()- 2 * turnController.getSetpoint());
    }

    public void driveStandard() {
	    myDrive.driveCartesian(Controller.getX(LeftHand), -Controller.getY(LeftHand), Controller.getX(RightHand));
	}

	private class DrivePID implements PIDOutput {
        @Override
        public void pidWrite(double output) {
            driveToRate = output;
        }
    }
}