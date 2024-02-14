/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.instant;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class SetIntakeArm extends InstantCommand {
  double pos;

  public SetIntakeArm(double pos) {
    super();
    requires(Robot.intake);
    this.pos = pos;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.intake.setTargetPosition(pos);
  }
}
