/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autonomous.AutoCommandGroup;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.Colour;
import frc.robot.subsystems.Outtake;

public class Robot extends TimedRobot {
	public static final double AUTO_TIMEOUT = 5;

	// Subsystem initialisation
	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Gyro gyro = new Gyro();
	public static final Climber climber = new Climber();
	public static final Lights lights = new Lights();
	public static OI oi;

	public static final Intake intake = new Intake();
	public static final Elevator elevator = new Elevator();
	public static final Outtake outtake = new Outtake();

	// Webcam declaration
	public static UsbCamera webcam;
	// public static UsbCamera jevois;

	// Autonomous setup
	public static AutoCommandGroup autonomous_command_group;

	public static enum AutonomousTarget {
		CARGO_FRONT, CARGO_1, CARGO_2, CARGO_3, ROCKET_NEAR, ROCKET_FAR, NOTHING
	}

	public static enum AutonomousStartPosition {
		LEFT, MID_LEFT, MID_RIGHT, RIGHT
	}

	public static enum AutonomousGamepiece {
		CARGO, HATCH, NOTHING
	}

	public static SendableChooser<AutonomousStartPosition> autonomous_position_chooser = new SendableChooser<AutonomousStartPosition>();
	public static SendableChooser<AutonomousTarget> autonomous_target_chooser = new SendableChooser<AutonomousTarget>();
	public static SendableChooser<AutonomousGamepiece> autonomous_gamepiece_chooser = new SendableChooser<AutonomousGamepiece>();

	// Jevois processing stuff
	public static int visionOffset = 0;
	private String t = "";
	private String[] in;
	// Used to track jevois serial status
	// 1 is added if available = 0
	// 1 is removed otherwise
	// If it = 0, serial port is made null
	private int errorCount = 0;
	private int periodicCount = 0;
	private int jevois_available = 0;

	@Override
	public void robotPeriodic() {
		periodicCount++;

		// Dashboard interface
		SmartDashboard.putNumber("Elevator Encoder", RobotMap.elevatorSpark1.getEncoder().getPosition());
		SmartDashboard.putNumber("Intake Angle", RobotMap.intakeArmTalon.getSelectedSensorPosition());
		SmartDashboard.putNumber("Current Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Current Absolute Gyro Angle", gyro.getAbsoluteAngle());
		// SmartDashboard.putBoolean("Camera connected", jevois.isConnected());
		// System.out.println(RobotMap.jevoisSerial.getBytesReceived());
		SmartDashboard.putBoolean("Serial good", RobotMap.jevoisSerial != null);
		SmartDashboard.putNumber("Vision Offset", visionOffset);
		// SmartDashboard.putNumber("Reverse", oi.shawnDrive.get() ? 1 : 0);
		// System.out.println(oi.shawnDrive.get());
		SmartDashboard.putNumber("LeftEncoder", RobotMap.leftOuttakeTalon.getSelectedSensorPosition());
		SmartDashboard.putNumber("RightEncoder", RobotMap.rightOuttakeTalon.getSelectedSensorPosition());
		SmartDashboard.putNumber("LeftSwitch", RobotMap.leftOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed() ? 1:0);
		SmartDashboard.putNumber("RightSwitch", RobotMap.rightOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed() ? 1:0);
		// VISION CODE REMOVED WITH REMOVAL OF JEVOIS
		// jevoisPeriodic();

		
		//System.out.print(RobotMap.leftOuttakeTalon.getClosedLoopTarget());
		//System.out.print(",  ");
		//System.out.print(RobotMap.rightOuttakeTalon.getClosedLoopTarget());
		//System.out.print(",  ");
		//System.out.print(RobotMap.leftOuttakeTalon.getSelectedSensorPosition());//getSensorCollection().getQuadraturePosition());
		//System.out.print(",  ");
 		//System.out.println(RobotMap.rightOuttakeTalon.getSelectedSensorPosition());//getSensorCollection().getQuadraturePosition());

	}

	public void jevoisPeriodic() {

		// if (RobotMap.jevoisSerial == null) {
		// 	if (periodicCount % 50 == 051) {
		// 		System.out.println("initializing JeVois Serial");
		// 		try {
		// 			RobotMap.jevoisSerial = new SerialPort(115200, Port.kUSB);
		// 		} catch (Exception e) {
		// 			System.out.println("JeVois error");
		// 			RobotMap.jevoisSerial = null;
		// 		}
		// 	}
		// } else {

		// 	jevois_available = RobotMap.jevoisSerial.getBytesReceived();
		// 	// Update collector
		// 	/*
		// 	 * int sum = available; for (int i = 0; i < collector.length - 1; i++) {
		// 	 * collector[i] = collector[i] + 1; sum += collector[i]; }
		// 	 * collector[collector.length - 1] = available;
		// 	 * 
		// 	 * // If there have been <2 bits in the last 5 messages, there's an issue if
		// 	 * (sum < 2) { offset = 0; }
		// 	 */

		// 	// Parse new offset
		// 	if (jevois_available == 0)
		// 		errorCount++;
		// 	else
		// 		errorCount = 0;

		// 	if (errorCount >= 150) {
		// 		System.out.println("Resetting Jevois serial");
		// 		RobotMap.jevoisSerial.reset();
		// 		RobotMap.jevoisSerial.close();
		// 		// RobotMap.jevoisSerial.setFlowControl(FlowControl.kRtsCts);
		// 		RobotMap.jevoisSerial = null;
		// 		errorCount = 0;
		// 	}

		// 	if (jevois_available > 0) {
		// 		try {
		// 			in = RobotMap.jevoisSerial.readString().split("\n");
		// 			// Process data
		// 			if (in.length > 2) {
		// 				t = in[in.length - 2];
		// 				if (t.length() > 1) {
		// 					// Remove the trailing whitespace
		// 					t = t.substring(0, t.length() - 1);
		// 					visionOffset = Integer.parseInt(t);
		// 				}
		// 			}
		// 		} catch (Exception e) {
		// 			// vision_offset = 0;
		// 			System.out.println("Parse Err");
		// 		}
		// 	}

		// }
	}

	public void enabledInit() {
		RobotMap.gyro.reset();
		Robot.lights.setColour(Lights.LEFT, Colour.PURPLE);
		Robot.lights.setColour(Lights.RIGHT, Colour.PURPLE);
	}

	@Override
	public void robotInit() {
		oi = new OI();
		oi.init();
		RobotMap.init();
		NetworkTableInstance.getDefault();
		gyro.calibrate();
		lights.setBoth(Lights.Colour.PURPLE);
		intake.init();
		// Setup dashboard
		autonomous_position_chooser.setDefaultOption("Left", AutonomousStartPosition.LEFT);
		autonomous_position_chooser.addOption("Mid Left", AutonomousStartPosition.MID_LEFT);
		autonomous_position_chooser.addOption("Mid Right", AutonomousStartPosition.MID_RIGHT);
		autonomous_position_chooser.addOption("Right", AutonomousStartPosition.RIGHT);
		SmartDashboard.putData("Start Position", autonomous_position_chooser);
		autonomous_target_chooser.setDefaultOption("Front", AutonomousTarget.CARGO_FRONT);
		autonomous_target_chooser.addOption("Cargo1", AutonomousTarget.CARGO_1);
		autonomous_target_chooser.addOption("Cargo2", AutonomousTarget.CARGO_2);
		autonomous_target_chooser.addOption("Cargo3", AutonomousTarget.CARGO_3);
		autonomous_target_chooser.addOption("Rocket near", AutonomousTarget.ROCKET_NEAR);
		autonomous_target_chooser.addOption("Rocket far", AutonomousTarget.ROCKET_FAR);
		autonomous_target_chooser.addOption("Nothing", AutonomousTarget.NOTHING);
		SmartDashboard.putData("Target", autonomous_target_chooser);
		autonomous_gamepiece_chooser.setDefaultOption("Nothing", AutonomousGamepiece.NOTHING);
		autonomous_gamepiece_chooser.addOption("Hatch", AutonomousGamepiece.HATCH);
		autonomous_gamepiece_chooser.addOption("Cargo", AutonomousGamepiece.CARGO);
		SmartDashboard.putData("Gamepiece", autonomous_position_chooser);

		// Setup webcam feed
		// webcam = CameraServer.getInstance().startAutomaticCapture(0);
		// webcam.setPixelFormat(PixelFormat.kYUYV);
		// webcam.setResolution(320, 240);
		// webcam.setFPS(25);

		// Setup jevois feed
		// jevois = CameraServer.getInstance().startAutomaticCapture(0);
		// jevois.setPixelFormat(PixelFormat.kYUYV);
		// jevois.setResolution(320, 240);
		// jevois.setFPS(25);

		/*
		 * try { RobotMap.jevoisSerial = new SerialPort(115200, Port.kUSB); } catch
		 * (Exception e) { System.out.println("JeVois error"); RobotMap.jevoisSerial =
		 * null; }
		 */

	}

	@Override
	public void disabledInit() {
		////
	}

	@Override
	public void disabledPeriodic() {
		// Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		enabledInit();
		autonomous_command_group = new AutoCommandGroup();
		autonomous_command_group.start();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.

		// Prevent old target height from causing issues
		RobotMap.elevatorSpark1.getPIDController().setReference(RobotMap.elevatorSpark1.getEncoder().getPosition(),
				ControlType.kPosition);
		//Robot.elevator.setTargetHeight(RobotMap.elevatorSpark1.getEncoder().getPosition(), 0, "Enable");
		Robot.elevator.setTargetHeight(Elevator.LOW_HEIGHT/2, 0, "Enable");

		intake.init();
		// TODO: preform the following command only when a button is pressed
		outtake.calibrate();
		enabledInit();
	}

	int c=0;
	@Override
	public void teleopPeriodic() {
		
		Scheduler.getInstance().run();
		if (Robot.outtake.leftCalibrated == 1)
		{
			Robot.outtake.leftCalibrated = 2;
			Robot.outtake.setSide(Outtake.SIDE_LEFT, Outtake.L_INTAKE);
		}
		if (Robot.outtake.rightCalibrated == 1)
		{
			Robot.outtake.rightCalibrated = 2;
			Robot.outtake.setSide(Outtake.SIDE_RIGHT, Outtake.R_INTAKE);
		}

		// if (++c%20==0) {
		// 	System.out.print(RobotMap.elevatorSpark1.getEncoder().getPosition());
		// 	System.out.print(",  ");
		// 	System.out.print(RobotMap.leftOuttakeTalon.getClosedLoopTarget());
		// 	System.out.print(",  ");
		// 	System.out.print(RobotMap.rightOuttakeTalon.getClosedLoopTarget());
		// 	System.out.print(",  ");
		// 	System.out.print(RobotMap.leftOuttakeTalon.getSelectedSensorPosition());//getSensorCollection().getQuadraturePosition());
		// 	System.out.print(",  ");
		// 	System.out.println(RobotMap.rightOuttakeTalon.getSelectedSensorPosition());//getSensorCollection().getQuadraturePosition());
		// }
	}
	@Override
	public void testInit() {
		RobotMap.leftOuttakeTalon.setSelectedSensorPosition(0);
		RobotMap.rightOuttakeTalon.setSelectedSensorPosition(0);
	}
	@Override
	public void testPeriodic() {
		/**
		 * ------------------------- CODE TO CALIBRATE OUTTAKE -------------------------
		 */
		RobotMap.leftOuttakeTalon.set(ControlMode.PercentOutput, Robot.oi.xbox.getJoyLeftY() / 3);
		RobotMap.rightOuttakeTalon.set(ControlMode.PercentOutput, Robot.oi.xbox.getJoyRightY() / 3);
		//System.out.println("L,R="+Robot.oi.xbox.getJoyLeftY() / 3+","+Robot.oi.xbox.getJoyRightY() / 3);

		if(RobotMap.leftOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed()){
			RobotMap.leftOuttakeTalon.setSelectedSensorPosition(0);
			//Robot.outtake.setSide(Outtake.SIDE_LEFT, 1);
		  }
		  
		  if(RobotMap.rightOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed()){
			RobotMap.rightOuttakeTalon.setSelectedSensorPosition(0);
			//Robot.outtake.setSide(Outtake.SIDE_RIGHT, 1);
		  }
		//if(RobotMap.leftOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed()){
		//	RobotMap.leftOuttakeTalon.setSelectedSensorPosition(0);
		//}

		//if(RobotMap.rightOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed()){
		//	RobotMap.rightOuttakeTalon.setSelectedSensorPosition(0);
		//}

		//System.out.println(RobotMap.leftOuttakeTalon.getSelectedSensorPosition() +"\t"+ RobotMap.rightOuttakeTalon.getSelectedSensorPosition());
		//System.out.println(RobotMap.leftOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed() +"\t"+ RobotMap.rightOuttakeTalon.getSensorCollection().isRevLimitSwitchClosed());

		if (Robot.oi.xbox.getButtonA()) {
			Robot.lights.setBoth(Lights.Colour.GREEN);
		}
		if (Robot.oi.xbox.getButtonX()) {
			Robot.lights.setBoth(Lights.Colour.BLUE);
		}
		if (Robot.oi.xbox.getButtonB()) {
			Robot.lights.setBoth(Lights.Colour.RED);
		}
		if (Robot.oi.xbox.getButtonY()) {
			Robot.lights.setBoth(Lights.Colour.OFF);
		}

		// RobotMap.intakeArmTalon.set(ControlMode.PercentOutput, Robot.oi.xbox.getJoyLeftX());
		// RobotMap.intakeArmVictor.set(ControlMode.PercentOutput, Robot.oi.xbox.getJoyLeftX());

	}
}