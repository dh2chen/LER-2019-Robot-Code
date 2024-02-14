/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Tools;
import frc.robot.subsystems.Elevator;

public class VisionDrive extends Command {

  public static final double AREA_1_FOOT = 0;
  public static final double AREA_2_FOOT = 0;
  public static final double AREA_3_FOOT = 0;
  public static final double AREA_TO_INCHES=(1/AREA_1_FOOT+2/AREA_2_FOOT+3/AREA_3_FOOT)/3;

  double lSpeed, rSpeed;

  final double MAX_DIFF = 20.0; 
  final double kP = 0.075;
  NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");

  boolean manual;
  public VisionDrive(boolean manual) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
    requires(Robot.elevator);
    this.manual = manual;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double offset = 0;
    double error = 0;

    if(nt.getEntry("tv").getDouble(0)==1){//If there is a target
      offset = nt.getEntry("tx").getDouble(0);
      error = getDistance();
    }


    lSpeed = Math.pow(offset / (MAX_DIFF), 3) * kP;

    rSpeed = -Math.pow(offset / (MAX_DIFF), 3) * kP;

    if(manual){
      lSpeed += 0.5*Tools.getAdaptedSpeed(Robot.oi.lJoy.getY());
      rSpeed += 0.5*Tools.getAdaptedSpeed(Robot.oi.rJoy.getY());
    }else{
      lSpeed += (error*error)/1000+0.3;
      rSpeed += (error*error)/1000+0.3;
    }
    //System.out.println(available+"\t"+offset+"\t"+lSpeed+"\t"+rSpeed);

    Robot.drivetrain.drive(Tools.fitToRange(-lSpeed, -1.0, 1.0), Tools.fitToRange(-rSpeed, -1.0, 1.0));
  }

  private double getDistance(){
    return nt.getEntry("ta").getDouble(0)*AREA_TO_INCHES;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(manual)
      return false;
    return (getDistance() == 0); //TODO: or limit switch pressed;
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
