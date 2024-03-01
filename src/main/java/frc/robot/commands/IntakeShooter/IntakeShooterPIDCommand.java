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

public class IntakeShooterPIDCommand extends Command {

    double upperRPM;
    double lowerRPM;


    private  final IntakeShooter m_IntakeShooter;


    public IntakeShooterPIDCommand(IntakeShooter m_IntakeShooter, double upperRPM, double lowerRPM) {
        this.m_IntakeShooter = m_IntakeShooter;
        this.upperRPM = upperRPM;
        this.lowerRPM = lowerRPM;
    }

    private void setShooterRPM(double upperRPM, double lowerRPM) 
    {   
        this.m_IntakeShooter.setShooterPIDEnabled(true);
        this.m_IntakeShooter.setShooterRPM(upperRPM, lowerRPM);
        
    }

    @Override
    public void initialize() 
    {
        
    }

  @Override
    public void execute() {

        
            if ((m_IntakeShooter.getState().getPosition() == IntakeShooterPositions.SHOOTCLOSE) || (m_IntakeShooter.getState().getPosition() == IntakeShooterPositions.SHOOTFAR) ) {
                setShooterRPM(upperRPM, lowerRPM);
            }



    }

  @Override
   public boolean isFinished(){
       return true;
   }





}
