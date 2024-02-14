/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands.instant;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Outtake;

/**
 * Add your docs here.
 */
public class SetOuttake extends InstantCommand {

  int side = 5;
  int position;
  public static final int SIDE_AUTO = 5;
  public static final int SIDE_BOTH = 6;

  public static int AUTO_OUT = -1;
  public static int AUTO_IN = -2;

  /**
   * Add your docs here.
   */
  public SetOuttake(int s, int pos) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Robot.outtake);
    side = s;
    position = pos;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {

    // Bring both in
    if (side == SIDE_BOTH) {
      if (position == AUTO_IN) {
        Robot.outtake.setSide(Outtake.SIDE_LEFT, Outtake.L_INTAKE);
        Robot.outtake.setSide(Outtake.SIDE_RIGHT, Outtake.R_INTAKE);
      }
      return;
    }

    // Auto-outtake using sensors
    if (side == SIDE_AUTO) {
      // Get distance from last tape
      double distance = (RobotMap.leftDriveSpark2.getEncoder().getPosition()
          + RobotMap.rightDriveSpark2.getEncoder().getPosition()) / 2.0;
      // If the last side was left, and we are close enough
      if (Robot.outtake.lastSide == Outtake.SIDE_LEFT && distance < Outtake.MAX_DIST) {
        // Set values
        side = Outtake.SIDE_LEFT;
        if (position == AUTO_OUT)
          position = Outtake.L_OUT;
        if (position == AUTO_IN)
          position = Outtake.L_INTAKE;
      }
      // If the last side was right, and we are close enough
      else if (Robot.outtake.lastSide == Outtake.SIDE_RIGHT && distance < Outtake.MAX_DIST) {
        // Set values
        side = Outtake.SIDE_RIGHT;
        if (position == AUTO_OUT)
          position = Outtake.R_OUT;
        if (position == AUTO_IN)
          position = Outtake.R_INTAKE;
      } else {
        return;
      }
    }
    Robot.outtake.setSide(side, position);
  }

}
