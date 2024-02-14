package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Tools;

/**
 *
 */
public class StraightGyroDriveCommand extends Command {

    public StraightGyroDriveCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.gyro.resetAngle();
        RobotMap.leftDriveSpark1.getEncoder().setPosition(0);
        RobotMap.rightDriveSpark1.getEncoder().setPosition(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double l = Tools.getAdaptedSpeed(Robot.oi.lJoy.getY())*0.7;
    	double r = Tools.getAdaptedSpeed(Robot.oi.rJoy.getY())*0.7;
    	
    	l = Math.abs(l) > Math.abs(r) ? l : r;
    	r = l;
    	
    	double[] straight_gyro_output = Robot.gyro.getStraightOutput(l, r);
    	
    	Robot.drivetrain.drive(-straight_gyro_output[1], -straight_gyro_output[0]);
    }

    protected boolean isFinished() {
        return false;
    }
}
