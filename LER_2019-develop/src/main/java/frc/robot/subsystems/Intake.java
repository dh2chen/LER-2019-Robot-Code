/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeCommand;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Competition bot:
  public static final double POSITION_MAX = 507;
  public static final double POSITION_UP = 350;
  public static final double POSITION_DOWN = 114;
  public static final double POSITION_MID = (POSITION_UP + POSITION_DOWN) / 2;
  /*
   * // Twin Bot: public static final double POSITION_MAX = 528; public static
   * final double POSITION_UP = 390; public static final double POSITION_DOWN =
   * 155; public static final double POSITION_MID = (POSITION_UP+POSITION_DOWN)/2;
   */
  public double targetPosition = POSITION_UP;

  final double SPEED = 0.60;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeCommand());
  }

  public void init() {
    if (getPosition() < POSITION_MID) {
      setTargetPosition(getPosition());
    } else {
      setTargetPosition(POSITION_UP);
    }
  }

  public void setTargetPosition(double position) {
    if (position > POSITION_MAX) { // sensor values are negative
      position = POSITION_MAX;
    }
    if (position < POSITION_DOWN) { // sensor values are negative
      position = POSITION_DOWN;
    }
    // Slow when going down to prevent damage
    if (position != POSITION_DOWN) {
      RobotMap.intakeArmTalon.config_kP(0, 3.0, 0);
      RobotMap.intakeArmTalon.config_kD(0, 0.2, 0);
      RobotMap.intakeArmTalon.config_kI(0, 0.000, 0);
    } else {
      RobotMap.intakeArmTalon.config_kP(0, 1, 0);
    }
    targetPosition = position;
    RobotMap.intakeArmTalon.set(ControlMode.Position, position);
  }

  public double getPosition() {
    return (RobotMap.intakeArmTalon.getSelectedSensorPosition());
  }

  public double getTargetPosition() {
    return (targetPosition);
  }

  public void spin(double speed) {
    RobotMap.intakeRollerTalon.set(ControlMode.PercentOutput, -speed);
  }

}
