package frc.robot.subsystems.utils;

import frc.robot.subsystems.utils.Enums.Position_Enums.IntakeShooterPositions;

public class IntakeShooterState {
    IntakeShooterPositions intakeShooterPosition;

    public IntakeShooterState() {
        this.intakeShooterPosition = IntakeShooterPositions.STOW;
    }

    public IntakeShooterState(IntakeShooterPositions intakeShooterStartingPosition) {
        this.intakeShooterPosition = intakeShooterStartingPosition;
    }
    
    public IntakeShooterPositions getPosition(){
        return this.intakeShooterPosition;
    }

    public void setPosition(IntakeShooterPositions intakeShooterPosition){
        this.intakeShooterPosition = intakeShooterPosition;
    }



    
    
}
