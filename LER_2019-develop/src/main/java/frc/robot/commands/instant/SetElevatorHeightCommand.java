/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.instant;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;

public class SetElevatorHeightCommand extends InstantCommand {

  double height;

  public SetElevatorHeightCommand(double targetHeight) {
    super();
    requires(Robot.elevator);
    height = targetHeight;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    // Set the height, with offset of 0
    Robot.elevator.setTargetHeight(height, 0, "Set");

    // Release grip if the height is low
    if (height < Elevator.LOW_HEIGHT) {
      Robot.outtake.setSide(Outtake.SIDE_LEFT, Outtake.L_INTAKE);
      Robot.outtake.setSide(Outtake.SIDE_RIGHT, Outtake.R_INTAKE);
    } else { // Grip the ball if elevator is raised
      Robot.outtake.setSide(Outtake.SIDE_LEFT, Outtake.L_GRIP);
      Robot.outtake.setSide(Outtake.SIDE_RIGHT, Outtake.R_GRIP);
    }
  }

}
