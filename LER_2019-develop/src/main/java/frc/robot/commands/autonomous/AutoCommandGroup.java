package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Robot.AutonomousStartPosition;
import frc.robot.commands.instant.SetElevatorHeightCommand;
import frc.robot.commands.instant.SetOuttake;
import frc.robot.commands.instant.Stop;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;

/**
 * Main autonomous command group for auto
 * <p>
 * There are 3 'Points' A,B, and C. <br>
 * <ul>
 * <b>A</b> is directly in front of the closest switch<br>
 * <b>B</b> is between the closest switch and scale, just a few feet ahead of
 * A<br>
 * <b>C</b> is between the far switch and scale, on the opposite side of the
 * field
 * <p>
 * </ul>
 * At each point the robot either continues to the next or breaks off to score.
 * 
 * @author Ewan
 *
 *
 */
public class AutoCommandGroup extends CommandGroup {
	public static final double SLOW_SPEED = 0.3;
	public static final double MEDIUM_SPEED = 0.6;
	public static final double HIGH_SPEED = 0.7;

	final double SPEED = 0.5;
	final double D_OFF_LEVEL2 = 20;
	final double D_SCORE_HATCH = 1;

	final double D_CLOSE = 14 * 12 + 4;
	final double D_MID = 16 * 12 + 1;
	final double D_FAR = 18 * 12;

	final int HATCH_FIRST = 1;
	final int INTAKE_FIRST = -1;

	public AutoCommandGroup() {
		Robot.AutonomousTarget target = Robot.autonomous_target_chooser.getSelected();
		Robot.AutonomousStartPosition startPosition = Robot.autonomous_position_chooser.getSelected();
		Robot.AutonomousGamepiece gamepiece = Robot.autonomous_gamepiece_chooser.getSelected();

		/*********************************************************************
		 * 
		 * First get off the hab
		 * 
		 *********************************************************************/

		int direction = INTAKE_FIRST;
		addSequential(new AutoGyroDriveStraightCommand(SLOW_SPEED, 1, .5));

		// From middle-left to left front cargoship
		//addSequential(new AutoGyroDriveCommand(SLOW_SPEED, 135, 2, false, 0));

		// From middle-right to right front cargoship
		//addSequential(new AutoGyroDriveCommand(SLOW_SPEED, 135, -2, false, 0));

		//// addParallel(new SetElevatorHeightCommand(Elevator.LOW_HEIGHT));	
		//addSequential(new AutoGyroDriveStraightCommand(SLOW_SPEED, 110, 3.5));
		//// addSequential(new SetElevatorHeightCommand(Elevator.GROUND_HEIGHT));
		//// addParallel(new AutoGyroDriveStraightCommand(SLOW_SPEED, 6, 1));

		// Off hab 1
		//addSequential(new AutoGyroDriveStraightCommand(MEDIUM_SPEED, 20));

		// Off hab 2
		//addSequential(new AutoGyroDriveStraightCommand(MEDIUM_SPEED, 86));//seems long because its airborne

		//To near left rocket
		//addSequential(new AutoGyroDriveCommand(MEDIUM_SPEED, 85, 80, false, 0));
		//addSequential(new AutoGyroDriveCommand(MEDIUM_SPEED, 35, 45, false, 0));

		// addSequential(new AutoGyroDriveStraightCommand(MEDIUM_SPEED, D_MID));

		//addSequential(new AutoGyroDriveStraightCommand(0.75, (D_MID - 45) * direction));
/*
		addSequential(new AutoGyroDriveStraightCommand(0.75, (D_MID - 45) * direction));
		
		addParallel(new SetElevatorHeightCommand(Elevator.LOW_HEIGHT));

		addSequential(new CurveDriveCommand(60*direction, -90, 0.50, false));
		addSequential(new Delay(500));
		addSequential(new AutoGyroDriveStraightCommand(0.25, 60*0.15));*/
	}

		// switch(startPosition){
		// 	case LEFT: //This acts like an OR
		// 	case RIGHT:
		// 		addSequential(new AutoGyroDriveCommand(0.5, 72));
		// 		break;
		// 	case MID_LEFT:
		// 	case MID_RIGHT:
		// 		break;
		// }


		/*********************************************************************
		 * 
		 * Line up with vision target or white line
		 * 
		 *********************************************************************/
	// 	if (gamepiece==Robot.AutonomousGamepiece.HATCH)
	// 	{
	// 		addSequential(new VisionDriveCommand());
	// 	} else if (gamepiece==Robot.AutonomousGamepiece.CARGO)
	// 	{
	// 		addSequential(new TapeAlignCommand());
	// 	} 

	// 	/*********************************************************************
	// 	 * 
	// 	 * Score gamepiece (either cargo or hatch)
	// 	 * 
	// 	 *********************************************************************/
	// 	if (gamepiece==Robot.AutonomousGamepiece.HATCH || target==Robot.AutonomousTarget.ROCKET_NEAR || target==Robot.AutonomousTarget.ROCKET_FAR)
	// 	{
	// 		addSequential(new SetElevatorHeightCommand(Elevator.LOW_HEIGHT));
	// 	} else {
	// 		addSequential(new SetElevatorHeightCommand(Elevator.CARGO_SHIP_HEIGHT));
	// 	}
	// 	switch (gamepiece) {
	// 		case CARGO:
	// 			if (startPosition == AutonomousStartPosition.LEFT || startPosition == AutonomousStartPosition.MID_LEFT)
	// 			{
	// 				addSequential(new SetOuttake(Outtake.SIDE_LEFT,Outtake.L_OUT));
	// 			} else {
	// 				addSequential(new SetOuttake(Outtake.SIDE_RIGHT,Outtake.R_OUT));
	// 			}
	// 			break;
	// 		case HATCH:
	// 			addSequential(new AutoGyroDriveCommand(SPEED/2, D_SCORE_HATCH));
	// 			addSequential(new SetElevatorHeightCommand(Elevator.LOW_HEIGHT-Elevator.BUMP_DOWN));
	// 			addSequential(new AutoGyroDriveCommand(SPEED/2, -D_SCORE_HATCH));
	// 			break;
	// 		default:
	// 	}
	// 	//addSequential(new Stop());
	// }
}
