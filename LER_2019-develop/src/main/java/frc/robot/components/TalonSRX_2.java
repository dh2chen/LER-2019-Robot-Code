package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Tools;

public class TalonSRX_2 extends TalonSRX {
    int minPosition;
    int maxPosition;

    public TalonSRX_2(int deviceNumber, int minPosition, int maxPosition) {
        super(deviceNumber);
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
    }

    public int getPosition() {
        return this.getSelectedSensorPosition(0);
    }

    public int getTargetPosition() {
        return this.getActiveTrajectoryPosition();
    }

    public void setTargetPosition(int position) {
        position = (int) Tools.fitToRange(position, minPosition, maxPosition);
        setSelectedSensorPosition(position, 0, 0);
    }
}

/*
2018 code
*/

/*
public double getPosition() {
    return RobotMap.lift_talon_1.getSelectedSensorPosition(0);
}

public double getTargetPosition() {
    return RobotMap.lift_talon_1.getActiveTrajectoryPosition();
}

public void setTargetPosition(double position) {
    position = Tools.maximum(position, MIN_POSITION);
    position = Tools.minimum(position, MAX_POSITION);
    RobotMap.lift_talon_1.setSelectedSensorPosition((int) position, 0, 0);
}
*/

/*
double target_position = Robot.lift.getTargetPosition();
if (Robot.oi.scale_height_button.get()) {
    target_position = Lift.SCALE_HEIGHT;
}
else if (Robot.oi.switch_height_button.get()) {
    target_position = Lift.SWITCH_HEIGHT;
}
else if (Robot.oi.ground_height_button.get()) {
    target_position = Lift.GROUND_HEIGHT;
}
target_position += Robot.oi.xbox.getJoyRightY() * MULTIPLIER;
Robot.lift.setTargetPosition(target_position);
*/