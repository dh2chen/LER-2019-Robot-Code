/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Tools;

public class DriveCommand extends Command {

  final double DEADZONE = 0.05;

  public DriveCommand() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Pushing the joysticks forward gives a negative Y value, whereas pushing them
    // backward gives a positive Y value
    double lSpeed = -Robot.oi.lJoy.getY();
    double rSpeed = -Robot.oi.rJoy.getY();
    double average = (lSpeed + rSpeed) / 2;

    //Deadzone
    if (Math.abs(lSpeed) < DEADZONE)
      lSpeed = 0;
    if (Math.abs(rSpeed) < DEADZONE)
      rSpeed = 0;

    //Get the speeds from the Tools function
    lSpeed = Tools.getAdaptedSpeed(lSpeed);
    rSpeed = Tools.getAdaptedSpeed(rSpeed);

    // if sticks are close and speed reasonable, go straight
    if (Math.abs(lSpeed - rSpeed) < 0.05 && Math.abs(average) > 0.25) {
      lSpeed = average;
      rSpeed = average;
    }
    //Slow and TURBO modes
    if (Robot.oi.slowDrive.get()) {
      Robot.drivetrain.drive(lSpeed * 0.25, rSpeed * 0.25);
    } else if (Robot.oi.TURBO.get()) {
      Robot.drivetrain.drive(lSpeed, rSpeed);
    } else {
      Robot.drivetrain.drive(lSpeed * 0.8, rSpeed * 0.8);
    }
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
