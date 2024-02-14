/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Outtake;
import frc.robot.RobotMap;

public class OuttakeCommand extends Command {
  public OuttakeCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.outtake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Set sides based on tape detection
    /*if(RobotMap.outerLeftSensor.isOnTape()){
      RobotMap.leftDriveSpark2.getEncoder().setPosition(0);
      RobotMap.rightDriveSpark2.getEncoder().setPosition(0);
      Robot.outtake.lastSide = Outtake.SIDE_LEFT;
    }   
    if(RobotMap.outerRightSensor.isOnTape()){
      RobotMap.leftDriveSpark2.getEncoder().setPosition(0);
      RobotMap.rightDriveSpark2.getEncoder().setPosition(0);
      Robot.outtake.lastSide = Outtake.SIDE_RIGHT;
    } */   

    //Reset last side if the robot has gone more than 3 feet
    if(RobotMap.leftDriveSpark2.getEncoder().getPosition() > 36 || RobotMap.rightDriveSpark2.getEncoder().getPosition() > 36){
      Robot.outtake.lastSide = Outtake.SIDE_NONE;
    }

    //TODO: Power monitoring when resetting to detect ball "DamonDrive"?

    if(RobotMap.leftOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed()){
      RobotMap.leftOuttakeTalon.setSelectedSensorPosition(0);
      Robot.outtake.setSide(Outtake.SIDE_LEFT, Outtake.L_INTAKE);
      if (Robot.outtake.leftCalibrated == 0)
        Robot.outtake.leftCalibrated = 1;
    }
    
    if(RobotMap.rightOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed()){
      RobotMap.rightOuttakeTalon.setSelectedSensorPosition(0);
      Robot.outtake.setSide(Outtake.SIDE_RIGHT, Outtake.R_INTAKE);
      if (Robot.outtake.rightCalibrated == 0)
        Robot.outtake.rightCalibrated = 1;
    }
    // if (mode==)
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
