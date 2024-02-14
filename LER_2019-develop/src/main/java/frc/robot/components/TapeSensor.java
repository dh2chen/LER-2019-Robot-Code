//  OPB732WZ Reflective Optical Sensor
//  This thing will switch from LOW to HIGH at 1.3 V. 

package frc.robot.components;

import edu.wpi.first.wpilibj.DigitalInput;

public class TapeSensor extends DigitalInput {
    boolean[] collector = {false, false, false, false};
    
    public TapeSensor(int channel) {
        super(channel);
        //System.out.println(channel);
    }

    //  this.get() will return false on tape and true otherwise because the voltage going to the RoboRIO is inverted in Jeremy's calibration circuit
    //  this.isOnTape() inverts this.get() and will return true on tape and false otherwise
    public boolean isOnTape() {
        double sum = 0;
        for(int i = 0; i<collector.length-1; i++){
            sum+=collector[i]?1:0;
            collector[i] = collector[i+1];
        }
        sum+=!get()?1:0;
        collector[collector.length-1] = !get();

        return sum/collector.length >= 0.4;
    }
}
