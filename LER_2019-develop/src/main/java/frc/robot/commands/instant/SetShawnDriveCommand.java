package frc.robot.commands.instant;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetShawnDriveCommand extends InstantCommand {

    public enum Mode{
        ON,OFF,TOGGLE
    }

    Mode mode;

    public SetShawnDriveCommand(Mode m) {
        super();
        requires(Robot.drivetrain);
        mode = m;
    }

    // Called once when the command executes
    protected void initialize() {
        switch(mode){
            case ON:
            
                Robot.drivetrain.shawnDriveIsActive = true;
                break;
            case OFF:
                Robot.drivetrain.shawnDriveIsActive = false;
                break;
            case TOGGLE:
                Robot.drivetrain.shawnDriveIsActive = !Robot.drivetrain.shawnDriveIsActive;
                break;
        }
    }
}
