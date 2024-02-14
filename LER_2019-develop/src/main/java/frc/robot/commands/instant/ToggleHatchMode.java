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

/**
 * Add your docs here.
 */
public class ToggleHatchMode extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ToggleHatchMode() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(Robot.elevator.getMode() == Elevator.Mode.CARGO){
      Robot.elevator.setMode(Elevator.Mode.HATCH);
    }
    if(Robot.elevator.getMode() == Elevator.Mode.HATCH){
      Robot.elevator.setMode(Elevator.Mode.CARGO);
    }
  }

}
