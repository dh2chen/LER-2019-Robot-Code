/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Lights;

public class Disco extends Command {
  public Disco() {
    requires(Robot.lights);
  }

  byte leftColour = 0b001;
  byte rightColour = 0b001;
  final byte RED = 0b100;
  final byte BLUE = 0b010;
  final byte GREEN = 0b001;
  int delay = 1;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lights.setColour(Lights.LEFT, (leftColour & RED) == RED, (leftColour & GREEN) == GREEN,
        (leftColour & BLUE) == BLUE);
    Robot.lights.setColour(Lights.RIGHT, (rightColour & RED) == RED, (rightColour & GREEN) == GREEN,
        (rightColour & BLUE) == BLUE);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (delay < 7) {
      delay++;
      return;
    }
    delay = 0;
    leftColour += 0b001;
    if (leftColour > 0b111)
      leftColour = 0b001;
    rightColour -= 0b001;
    if (rightColour == 0b000)
      rightColour = 0b111;

    Robot.lights.setColour(Lights.LEFT, (leftColour & RED) == RED, (leftColour & GREEN) == GREEN,
        (leftColour & BLUE) == BLUE);
    Robot.lights.setColour(Lights.RIGHT, (rightColour & RED) == RED, (rightColour & GREEN) == GREEN,
        (rightColour & BLUE) == BLUE);
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
