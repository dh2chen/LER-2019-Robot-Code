/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Tools;

public class TapeAlignCommand extends Command {

  boolean engageLock = false;
  boolean passedTape = false;

  public TapeAlignCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drivetrain.resetEncoderPositions();
    engageLock = false;
    passedTape = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    //System.out.println("Aligning: "+engageLock);

    if((RobotMap.outerLeftSensor.isOnTape() || RobotMap.innerLeftSensor.isOnTape()) || 
        (RobotMap.outerRightSensor.isOnTape() || RobotMap.innerRightSensor.isOnTape())){
      engageLock = true;
      RobotMap.leftDriveSpark1.getEncoder().setPosition(0);
      RobotMap.rightDriveSpark1.getEncoder().setPosition(0);
    }

    if(engageLock){
      RobotMap.leftDriveSpark1.getPIDController().setReference(0, ControlType.kPosition);
      RobotMap.rightDriveSpark1.getPIDController().setReference(0, ControlType.kPosition);
    }else{
      normalDrive();
    }
  }

  public void normalDrive(){
        //  Pushing the joysticks forward gives a negative Y value, whereas pushing them backward gives a positive Y value
        double lSpeed = -Robot.oi.lJoy.getY();
        double rSpeed = -Robot.oi.rJoy.getY();
        double average = (lSpeed+rSpeed)/2;
    
        lSpeed = Tools.getAdaptedSpeed(lSpeed)*0.5;
        rSpeed = Tools.getAdaptedSpeed(rSpeed)*0.5;
    
        // if sticks are close and speed reasonable, go straight
        if(Math.abs(lSpeed-rSpeed)<0.05 && Math.abs(average)>0.25){
          lSpeed = average;
          rSpeed = average;
        }
        if(Robot.oi.slowDrive.get()){
          Robot.drivetrain.drive(lSpeed*0.5, rSpeed*0.5);
        }
        else{
          Robot.drivetrain.drive(lSpeed, rSpeed);
        }
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
