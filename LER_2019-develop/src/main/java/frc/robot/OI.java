/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Disco;
import frc.robot.commands.StraightGyroDriveCommand;
import frc.robot.commands.autonomous.AutoIntakeCommand;
import frc.robot.commands.autonomous.TapeAlignCommand;
import frc.robot.commands.autonomous.VisionDrive;
import frc.robot.commands.instant.SetElevatorHeightCommand;
import frc.robot.commands.instant.SetIntakeArm;
import frc.robot.commands.instant.SetOuttake;
import frc.robot.commands.instant.Stop;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class OI {

	// input number definitions
	final int R_JOY = 0;
	final int L_JOY = 1;

	final int XBOX = 2;

	// Left Joystick buttons
	final int GYRO_DRIVE_BUTTON = 1;
	final int VISION_DRIVE_BUTTON = 3;
	final int RESET_CAM_BUTTON = 10;
	final int DISCO_BUTTON = 2;
	final int DEFENSE_BUTTON = 7;
	final int LEFT_OUT = 8;
	final int LEFT_IN = 9;

	// Right Joystick buttons
	final int SLOW_DRIVE_BUTTON = 2;
	final int TURBO_BUTTON = 3;
	final int SHAWN_DRIVE_BUTTON = 5;
	final int TAPE_DRIVE_BUTTON = 1;
	final int RIGHT_OUT = 8;
	final int RIGHT_IN = 9;
	final int FIX_DT = 4;

	// Xbox buttons:
	final int GROUND_HEIGHT_BUTTON = XBoxController.XBOX_A;
	final int LOW_ROCKET_BUTTON = XBoxController.XBOX_X;
	final int MID_ROCKET_BUTTON = XBoxController.XBOX_Y;
	final int HIGH_ROCKET_BUTTON = XBoxController.XBOX_B;
	final int CARGO_SHIP_HEIGHT_BUTTON = XBoxController.XBOX_RB;

	final int ELEVATOR_SHIFT_UP = XBoxController.XBOX_RIGHT_TRIGGER;
	final int ELEVATOR_SHIFT_DOWN = XBoxController.XBOX_LEFT_TRIGGER;

	final int INTAKE_BUTTON = XBoxController.XBOX_START;
	final int INTAKE_CANCEL_BUTTON = XBoxController.XBOX_BACK;

	final int OUTTAKE_LEFT_BUTTON = XBoxController.XBOX_L3;
	final int OUTTAKE_RIGHT_BUTTON = XBoxController.XBOX_R3;
	final int AUTO_OUTTAKE_BUTTON = XBoxController.XBOX_LB;

	// Initialisation
	public Joystick lJoy = new Joystick(L_JOY);
	public Joystick rJoy = new Joystick(R_JOY);
	public XBoxController xbox = new XBoxController(XBOX);

	// LEFT
	JoystickButton gyroDrive = new JoystickButton(lJoy, GYRO_DRIVE_BUTTON);
	JoystickButton visionDrive = new JoystickButton(lJoy, VISION_DRIVE_BUTTON);
	JoystickButton resetCam = new JoystickButton(lJoy, RESET_CAM_BUTTON);
	JoystickButton disco = new JoystickButton(lJoy, DISCO_BUTTON);
	JoystickButton defense = new JoystickButton(lJoy, DEFENSE_BUTTON);
	public JoystickButton leftIn = new JoystickButton(lJoy, LEFT_IN);
	public JoystickButton leftOut = new JoystickButton(lJoy, LEFT_OUT);

	// RIGHT
	public JoystickButton slowDrive = new JoystickButton(rJoy, SLOW_DRIVE_BUTTON);
	public JoystickButton shawnDrive = new JoystickButton(rJoy, SHAWN_DRIVE_BUTTON);
	public JoystickButton TURBO = new JoystickButton(rJoy, TURBO_BUTTON);
	JoystickButton tapeDrive = new JoystickButton(rJoy, TAPE_DRIVE_BUTTON);
	public JoystickButton rightIn = new JoystickButton(lJoy, RIGHT_IN);
	public JoystickButton rightOut = new JoystickButton(lJoy, RIGHT_OUT);
	JoystickButton fixDT = new JoystickButton(lJoy, FIX_DT);

	// XBOX
	XBoxButton groundHeight = new XBoxButton(xbox, GROUND_HEIGHT_BUTTON);
	XBoxButton lowHeight = new XBoxButton(xbox, LOW_ROCKET_BUTTON);
	XBoxButton midHeight = new XBoxButton(xbox, MID_ROCKET_BUTTON);
	XBoxButton highHeight = new XBoxButton(xbox, HIGH_ROCKET_BUTTON);
	XBoxButton cargoShipHeight = new XBoxButton(xbox, CARGO_SHIP_HEIGHT_BUTTON);

	XBoxButton intake = new XBoxButton(xbox, INTAKE_BUTTON);
	XBoxButton cancelIntake = new XBoxButton(xbox, INTAKE_CANCEL_BUTTON);

	XBoxButton outtakeLeft = new XBoxButton(xbox, OUTTAKE_LEFT_BUTTON);
	XBoxButton outtakeRight = new XBoxButton(xbox, OUTTAKE_RIGHT_BUTTON);
	XBoxButton autoOuttake = new XBoxButton(xbox, AUTO_OUTTAKE_BUTTON);

	//Right
	public XBoxTrigger shiftUp = new XBoxTrigger(xbox, false);
	//Left
	public XBoxTrigger shiftDown = new XBoxTrigger(xbox, true);

	// XBoxButton disco = new XBoxButton(xbox, DISCO_BUTTON);

	public void init() {
		// Joystick Buttons
		gyroDrive.whileHeld(new StraightGyroDriveCommand());
		visionDrive.whileHeld(new VisionDrive(false));
		tapeDrive.whileHeld(new TapeAlignCommand());
		defense.whenPressed(new SetIntakeArm(Intake.POSITION_MAX));

		// XBox Buttons
		groundHeight.whenPressed(new SetElevatorHeightCommand(Elevator.GROUND_HEIGHT));
		lowHeight.whenPressed(new SetElevatorHeightCommand(Elevator.LOW_HEIGHT));
		midHeight.whenPressed(new SetElevatorHeightCommand(Elevator.MID_HEIGHT));
		highHeight.whenPressed(new SetElevatorHeightCommand(Elevator.HIGH_HEIGHT));
		cargoShipHeight.whenPressed(new SetElevatorHeightCommand(Elevator.CARGO_SHIP_HEIGHT));

		intake.whenPressed(new AutoIntakeCommand());
		cancelIntake.whenPressed(new SetIntakeArm(Intake.POSITION_UP));

		fixDT.whenPressed(new Stop());

		outtakeLeft.whenPressed(new SetOuttake(Outtake.SIDE_LEFT, Outtake.L_OUT));
		outtakeLeft.whenReleased(new SetOuttake(Outtake.SIDE_LEFT, Outtake.L_GRIP));
		outtakeRight.whenPressed(new SetOuttake(Outtake.SIDE_RIGHT, Outtake.R_OUT));
		outtakeRight.whenReleased(new SetOuttake(Outtake.SIDE_RIGHT, Outtake.R_GRIP));

		autoOuttake.whenPressed(new SetOuttake(SetOuttake.SIDE_AUTO, SetOuttake.AUTO_OUT));
		autoOuttake.whenReleased(new SetOuttake(Outtake.SIDE_LEFT, Outtake.L_INTAKE));
		autoOuttake.whenReleased(new SetOuttake(Outtake.SIDE_RIGHT, Outtake.R_INTAKE));

		disco.whileHeld(new Disco());
	}
}
