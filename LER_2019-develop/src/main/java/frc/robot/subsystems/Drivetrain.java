/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.Tools;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain extends Subsystem {
	public boolean shawnDriveIsActive = false;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveCommand());
	}
	
	public void drive(double l, double r) {
		//Other motors are followers
		if(Robot.oi.shawnDrive.get()){
			RobotMap.leftDriveSpark1.set(r);
			RobotMap.rightDriveSpark1.set(-l);
			return;
		}
		//Code for safety at demos:
		l*=0.8;
		r*=0.8;

		RobotMap.leftDriveSpark1.set(l);
		RobotMap.rightDriveSpark1.set(-r);  //  r is inverted because the left and right motors are oriented in opposite directions
	}

	public void setPercentVoltage(double l, double r) {
		RobotMap.leftDriveSpark1.set(l);	// because talons 2 and 3 follow 1, we only need to set 1
		RobotMap.rightDriveSpark1.set(-r);
	}
	public double getLeftEncoderPosition() {
		return RobotMap.leftDriveSpark1.getEncoder().getPosition();
	}

	public double getRightEncoderPosition() {
		return -RobotMap.rightDriveSpark1.getEncoder().getPosition();
	}

	public double getAverageEncoderPosition() {
		return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
	}

	public void resetEncoderPositions() {
		RobotMap.leftDriveSpark1.getEncoder().setPosition(0);
		RobotMap.rightDriveSpark1.getEncoder().setPosition(0);
	}

	public Object[] getAutoDriveOutput(double speed, double distance, double time) {
		//	TODO: Update these values
		final double SLOW_DOWN_THRESHOLD = 15.75;	//	Originally 0.4 m or 40 cm, now 15.75"
    	final double MIN_SPEED = 0.2;
		final double FINISHED_TOLERANCE = 2;		//	Originally 0.05 m or 5 cm, now 2"
		
    	boolean finished = false;
    	double l = 0;
    	double r = 0;
		/*
		double averageEncoderPosition = time;	//	Unsure as to why time is being used in place of the encoders; perhaps because it's a good approximation? Or were the encoders having issues?
		*/
		double averageEncoderPosition = getAverageEncoderPosition();
		double distanceLeft = distance - averageEncoderPosition;
		//System.out.println(getLeftEncoderPosition() +"\t"+ getRightEncoderPosition()+"\t"+distance + "\t" + distanceLeft);

		SmartDashboard.putNumber("Distance Left", distanceLeft);
    	if (distanceLeft > SLOW_DOWN_THRESHOLD) {
    		l = speed;
    	} else if (distanceLeft < -SLOW_DOWN_THRESHOLD) {
    		l = -speed;
		} else {
			l = speed * (distanceLeft / SLOW_DOWN_THRESHOLD);
			l = Tools.setAbsoluteMinimum(l, MIN_SPEED);
		}
		   	

    	// if (Math.abs(distanceLeft) < FINISHED_TOLERANCE || ((RobotMap.leftTapeSensor2.get() || RobotMap.rightTapeSensor2.get()) && line_stop)) {
		// 	finished = true;
		// }
				
		if (Math.abs(distanceLeft) < FINISHED_TOLERANCE) {
			finished = true;
		}
    	
    	r = l;
    	
    	return new Object[] {l, r, finished};
	}	
}
