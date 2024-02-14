/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class AutoIntakeCommand extends Command {

  final double SPEED = 0.47;
  final double DEADZONE = 0.2;
  boolean pressed = false;
  long time = 0;

  public AutoIntakeCommand() {
    requires(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.outtake.setSide(Outtake.SIDE_LEFT, Outtake.L_INTAKE);
    Robot.outtake.setSide(Outtake.SIDE_RIGHT, Outtake.R_INTAKE);
    Robot.intake.setTargetPosition(Intake.POSITION_DOWN);
    pressed = false;
    Robot.elevator.setTargetHeight(Elevator.GROUND_HEIGHT, 0, "INTAKE");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Set the roller speed
    Robot.intake.spin(SPEED);

    // First time the switch is pressed
    if (!RobotMap.intakeLimitSwitch.get() && !pressed) {
      pressed = true;
      time = System.currentTimeMillis();
    }
    // Wait 250ms after press to retract arm
    if (pressed && System.currentTimeMillis() - time > 250) {
      Robot.intake.setTargetPosition(Intake.POSITION_UP);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // It's finished when the button is pressed and the target is up
    return (Robot.intake.getTargetPosition() == Intake.POSITION_UP && RobotMap.intakeLimitSwitch.get());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Stop roller, ensure the target is up
    Robot.intake.spin(0);
    Robot.intake.setTargetPosition(Intake.POSITION_UP);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
