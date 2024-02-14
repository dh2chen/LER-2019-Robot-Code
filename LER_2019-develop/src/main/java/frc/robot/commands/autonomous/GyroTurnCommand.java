package frc.robot.commands.autonomous;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GyroTurnCommand extends Command {
	private double target_angle;
	private boolean finished = false;
	
    public GyroTurnCommand(double target_angle) {
        // Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		//	TODO: Figure out what the following line means and (possibly) change its value
    	double GYRO_MULTIPLIER = 5.475d / 360d;	// gyro tends to increase in error as it turns, this compensates
    	requires(Robot.drivetrain);
    	this.target_angle = target_angle;
    	this.target_angle -= target_angle * GYRO_MULTIPLIER;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.gyro.resetAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	final double HIGH_SPEED = 0.35;
    	final double LOW_SPEED = 0.25;
    	final double THRESHOLD_ANGLE = 30;
    	final double TOLERANCE = 1.5;
    	double l = 0;
    	double r = 0;
    	
    	double current_angle = Robot.gyro.getAngle();
    	double angle_difference = current_angle - target_angle;
    	
    	if (angle_difference > THRESHOLD_ANGLE) {
    		l = HIGH_SPEED;
    		r = -HIGH_SPEED;
    	}
    	else if (angle_difference < -THRESHOLD_ANGLE) {
    		l = -HIGH_SPEED;
    		r = HIGH_SPEED;
    	}
    	else if (angle_difference > TOLERANCE && angle_difference <= THRESHOLD_ANGLE) {
    		l = LOW_SPEED;
    		r = -LOW_SPEED;
    	}
    	else if (angle_difference < -TOLERANCE && angle_difference >= -THRESHOLD_ANGLE) {
    		l = -LOW_SPEED;
    		r = LOW_SPEED;
    	}
    	else if (Math.abs(angle_difference) < TOLERANCE) {
    		finished = true;
    	}
    	Robot.drivetrain.drive(l, r);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
