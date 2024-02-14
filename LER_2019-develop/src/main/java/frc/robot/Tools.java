package frc.robot;

public class Tools {

	static double DEADZONE = 0.02;

	public static double getAdaptedSpeed(double speed) {
		if (Math.abs(speed) < DEADZONE)
			speed = 0;
		if (Robot.oi.shawnDrive.get()) {
			speed = -speed;
		}
		double out = (0.5 * (Math.sin(Math.PI * speed - Math.PI / 2)) + 0.5);
		// if(out < 0.1) out = 0;
		return out * (speed > 0 ? 1 : -1);
		// OLD: (0.5*(Math.sin(Math.PI*speed-Math.PI/2))+0.5)
		// NEW: 0.25*Math.sin((6*Math.PI/5)*speed -3*Math.PI/5)+0.5*speed+0.25
	}

	public static double fitToRange(double value, double min, double max) {
		value = value < min ? min : value;
		value = value > max ? max : value;
		return value;
	}

	public static double setAbsoluteMinimum(double value, double min) {
		value = (value < min && value > 0) ? min : value;
		value = (value > -min && value < 0) ? -min : value;
		return value;
	}

	public static double getTimeSeconds() {
		return (double) System.currentTimeMillis() / 1000.0;
	}

	public static double closestEquivalentAngle(double target_angle) {
		double t = target_angle;
		double c = Robot.gyro.getAbsoluteAngle();
		boolean b = true;
		while (b) {
			if (Math.abs(c - t) > Math.abs(c - (t + 360))) {
				t += 360;
			} else if (Math.abs(c - t) > Math.abs(c - (t - 360))) {
				t -= 360;
			} else {
				b = false;
			}
		}
		return t;
	}
}
