/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class LockDriveCommand extends Command {
  public LockDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.leftDriveSpark1.getPIDController().setP(1);
    RobotMap.leftDriveSpark1.getPIDController().setOutputRange(-1, 1);
    RobotMap.leftDriveSpark1.getPIDController().setReference(RobotMap.leftDriveSpark1.getEncoder().getPosition(), ControlType.kPosition);
    
    
    RobotMap.rightDriveSpark1.getPIDController().setP(1);
    RobotMap.rightDriveSpark1.getPIDController().setOutputRange(-1, 1);
    RobotMap.rightDriveSpark1.getPIDController().setReference(RobotMap.rightDriveSpark1.getEncoder().getPosition(), ControlType.kPosition);
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // System.out.print(RobotMap.leftDriveSpark1.getEncoder().getPosition());
    // System.out.print("\t");
    // System.out.println(RobotMap.rightDriveSpark1.getEncoder().getPosition());

    //TODO: Allow the driver to move the bot


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
