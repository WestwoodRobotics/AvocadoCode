package frc.robot.subsystems.utils;

import frc.robot.subsystems.utils.Enums.Position_Enums.IntakeShooterPositions;

public class IntakeShooterState {
    IntakeShooterPositions intakeShooterPosition;

    public IntakeShooterState(IntakeShooterPositions stow) {
        this.intakeShooterPosition = stow;
    }
    
    public IntakeShooterPositions getPosition(){
        return intakeShooterPosition;
    }

    public void setState(IntakeShooterPositions intakeShooterPosition){
        this.intakeShooterPosition = intakeShooterPosition;
    }



    
    
}
