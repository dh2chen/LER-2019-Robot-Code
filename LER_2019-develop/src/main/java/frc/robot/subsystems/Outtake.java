/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.OuttakeCommand;

/**
 * Add your docs here.
 */
public class Outtake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // TODO: re-calibrate grip, intake, and out positions for bot right and left
  public static final int L_CALIBRATE = -2000;
  public static final int L_GRIP = 200;
  public static final int L_INTAKE = 300;
  public static final int L_OUT = 900;
  
  public static final int R_CALIBRATE = -2000;
  public static final int R_GRIP = 100;
  public static final int R_INTAKE = 200;
  public static final int R_OUT = 800;
  

  public static final int SIDE_RIGHT = 1;
  public static final int SIDE_LEFT = 0;
  public static final int SIDE_NONE = -1;

  /***The maxium distance from the line, in inches */
  public static final int MAX_DIST = 4;

  int lTarget = -1;

  int rTarget = -1;

  public int lastSide = SIDE_NONE;
  public int leftCalibrated = 0;
  public int rightCalibrated = 0;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new OuttakeCommand());
  }

  // Sets the outtake to a large negative position until it hits limit switch
  public void calibrate() {
    RobotMap.leftOuttakeTalon.set(ControlMode.Position, L_CALIBRATE);
    RobotMap.rightOuttakeTalon.set(ControlMode.Position, R_CALIBRATE);
  }

  public void setSide(int side, int target) {
    if (side == SIDE_LEFT) {
      lTarget = target;
    }

    if (side == SIDE_RIGHT) {
      rTarget = target;
    }

    RobotMap.leftOuttakeTalon.set(ControlMode.Position, lTarget);
    RobotMap.rightOuttakeTalon.set(ControlMode.Position, rTarget);
  }
}
