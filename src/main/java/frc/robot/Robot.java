/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot implements PIDOutput {
	Hand LeftHand = GenericHID.Hand.kLeft;
	Hand RightHand = GenericHID.Hand.kRight;
	XboxController Controller = new XboxController(0);
	static AHRS ahrs = new AHRS(SPI.Port.kMXP);
	static double kP = 0.05;
	static double kI = 0.00;
	static double kD = 0.00;
	static double kF = 0.00;
	static double kToleranceDegrees = 2.0f;
	PIDController turnController;
	// Talons:
	Talon four = new Talon(3);
	Talon one = new Talon(2);
	Spark three = new Spark(1);
	Spark two = new Spark(0);
	double rotateToAngleRate;
	double Target = 0;
	// Drive Base
	MecanumDrive myDrive = new MecanumDrive(one, four, two, three);


	public void robotInit() {
		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-.75, -.75);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		myDrive.setDeadband(.1);
	}
	
    public void teleopInit(){
		turnController.setSetpoint(Target);
		turnController.enable();
	}

	public void teleopPeriodic() {
		turnController.setP(kP);
		turnController.setI(kI);
		turnController.setD(kD);
		turnController.setF(kF);
		kP = SmartDashboard.getNumber("kP", .05);
		kI = SmartDashboard.getNumber("kI", .00);
		kD = SmartDashboard.getNumber("kD", .00);
		kF = SmartDashboard.getNumber("kF", .00);
		SmartDashboard.putNumber("kP", kP);
		SmartDashboard.putNumber("kI", kI);
		SmartDashboard.putNumber("kD", kD);
		SmartDashboard.putNumber("kF", kF);

		if ( Controller.getRawButton(1)) {
            ahrs.reset();
        }
		if(Controller.getPOV()== 0) Target = 0;
		if(Controller.getPOV()== 90) Target = 90;
		if(Controller.getPOV()== 180) Target = 180;
		if(Controller.getPOV()== 270) Target = -90;
        turnController.setSetpoint(Target);
        myDrive.driveCartesian(Controller.getX(LeftHand), Controller.getY(LeftHand), rotateToAngleRate, ahrs.getAngle());
	}


	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
		
	}

}
