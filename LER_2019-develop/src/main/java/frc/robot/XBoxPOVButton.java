package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class XBoxPOVButton extends Button {
	public int angle;
	XBoxController xbox;
	
	public XBoxPOVButton(XBoxController xbox, int angle) {
		this.angle = angle;
		this.xbox = xbox;
	}
	@Override
	public boolean get() {
		return xbox.getPOV() == angle;
	}
}
