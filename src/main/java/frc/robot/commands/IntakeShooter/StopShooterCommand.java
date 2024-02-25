package frc.robot.commands.IntakeShooter;

import com.fasterxml.jackson.databind.node.BooleanNode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Enums.Position_Enums.IntakeShooterPositions;

public class StopShooterCommand extends Command {

    double upperRPM;
    Timer timer;
    double lowerRPM;
    

    private  final IntakeShooter m_IntakeShooter;
    public StopShooterCommand(IntakeShooter m_IntakeShooter) {
        this.m_IntakeShooter = m_IntakeShooter;
        m_IntakeShooter.stopShooter();
    }

    private void setShooterRPM(double upperRPM, double lowerRPM) 
    {   
        this.m_IntakeShooter.setShooterPIDEnabled(true);
        this.m_IntakeShooter.setShooterRPM(upperRPM, lowerRPM);
        timer = new Timer();
        timer.reset();
    }

    @Override
    public void initialize() 
    {
        
    }

  @Override
    public void execute() {
    }

  @Override
   public boolean isFinished(){
        return false;
   }





}
