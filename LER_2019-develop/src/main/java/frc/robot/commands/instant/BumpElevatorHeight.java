/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.instant;

import java.time.format.SignStyle;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/**
 * Add your docs here.
 */
public class BumpElevatorHeight extends InstantCommand {
  public static final boolean UP = true;
  public static final boolean DOWN = false;

  boolean direction;

  /**
   * Add your docs here.
   */
  public BumpElevatorHeight(boolean direction) {
    super();
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);

    this.direction = direction;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    double height = Robot.elevator.getTargetHeight();
    //System.out.println(height);

    /*if(Robot.intakeArm.getTargetPosition()!=IntakeArm.POSITION_DOWN){
      Robot.intakeArm.setTargetPosition(IntakeArm.POSITION_DOWN);
    }*/


    if (height > Elevator.MAX_HEIGHT && direction == UP) {
      return;
    }
    if (height < Elevator.GROUND_HEIGHT && direction == DOWN) {
      return;
    }

    //System.out.println(direction == DOWN);

    for (int i = 0; i < Elevator.HEIGHTS.length; i++) {
      try {
        // If step is lower then current, and next step is higher
        if (height >= Elevator.HEIGHTS[i] && height <= Elevator.HEIGHTS[i + 1]) {
          if (direction == UP && height == Elevator.HEIGHTS[i + 1])
            continue;

          //System.out.println(height + " is between " + Elevator.HEIGHTS[i] + " and " + Elevator.HEIGHTS[i + 1]);
          if (direction == UP) {
            //System.out.println("UP to" + Elevator.HEIGHTS[i + 1]);
            // Go to higher step
            Robot.elevator.setTargetHeight(Elevator.HEIGHTS[i + 1], 0, "Bump up");
            break;
          } else if (direction == DOWN) {
            // Go to lower step
            //System.out.println("DOWN to" + Elevator.HEIGHTS[i]);
            Robot.elevator.setTargetHeight(Elevator.HEIGHTS[i], 0, "Bump down");
            break;
          }
        }
      } catch (Exception e) {

      }
    }
  }

}
