package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak {
    private DigitalInput input;
    public BeamBreak(int channel){
       input = new DigitalInput(channel);
    }

    public boolean getStatus(){
        return (input.get());
    }
}
