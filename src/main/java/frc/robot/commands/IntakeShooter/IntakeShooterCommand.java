package frc.robot.commands.IntakeShooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeShooter.IntakeShooter;
import frc.robot.subsystems.swerve.SwerveDrive;
// import frc.robot.subsystems.swerve.DriveSpeed;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

// import static frc.robot.Constants.DriveConstants.kMaxAngularSpeed;
// import static frc.robot.Constants.DriveConstants.kMaxSpeedMetersPerSecond;

public class IntakeShooterCommand extends Command {

  private final IntakeShooter m_IntakeShooter;








  public IntakeShooterCommand(IntakeShooter m_IntakeShooter) {
    this.m_IntakeShooter = m_IntakeShooter;

    addRequirements(m_IntakeShooter);
  }


  @Override
  public void initialize() 
  {

  }



  public Command setIntakeShooterPower(double power) {
    Command command = new InstantCommand (() -> m_IntakeShooter.setIntakeShooterPower(power), m_IntakeShooter);
    return command;
  }

  public Command setIntakePivotPower(double power) {
    Command command = new InstantCommand (() -> m_IntakeShooter.setIntakePivotPower(power), m_IntakeShooter);
    return command;
  }
  
  public Command stopIntakeShooter(){
    Command command = new InstantCommand (() -> m_IntakeShooter.stopIntakeShooter(), m_IntakeShooter);
    return command;
  }

  public Command stopIntakePivot(){
    Command command = new InstantCommand (() -> m_IntakeShooter.stopIntakePivot(), m_IntakeShooter);
    return command;
  }

  public Command stowIntakePivot(){
    Command command = new InstantCommand(() -> m_IntakeShooter.setIntakePivotPosition(IntakeShooterPositions.STOW, IntakeShooterConstants.kIntakePivotff), m_IntakeShooter);
    return command;
  }

  public Command engageIntakePivot(){
    Command command = new InstantCommand(() -> m_IntakeShooter.setIntakePivotPosition(IntakeShooterPositions.ENGAGE, IntakeShooterConstants.kIntakePivotff), m_IntakeShooter);
    return command;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }


  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // should never end in teleop
    return false;
  }
}