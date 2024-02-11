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

public class IntakeShooterCommandFactory {

  private final IntakeShooter m_IntakeShooter;

  public IntakeShooterCommandFactory(IntakeShooter m_IntakeShooter) {
    this.m_IntakeShooter = m_IntakeShooter;
  }

  public Command setIntakeShooterPower(double power) {
    Command command = new InstantCommand (() -> m_IntakeShooter.setIntakeShooterPower(power), m_IntakeShooter);
    return command;
  }

  public Command setIntakePivotPower(double power) {
    Command command = new InstantCommand (() -> m_IntakeShooter.setIntakePivotPower(power), m_IntakeShooter);
    return command;
  }

  private void stopIntakeImpl() {
    this.m_IntakeShooter.setIntakeShooterPower(0);
    this.m_IntakeShooter.setIntakeStowPower(0);
  }

  public Command stopIntake() {
    InstantCommand command = new InstantCommand(() -> stopIntakeImpl());
    command.addRequirements(this.m_IntakeShooter);
    return command;
  }

  private void manualPivotImpl(double power) {
    this.m_IntakeShooter.setIntakePivotPower(power);
    this.m_IntakeShooter.setPivotPIDEnabled(false);
  }

  public Command manualPivot(double power) {
    InstantCommand command = new InstantCommand(() -> manualPivotImpl(power));
    command.addRequirements(this.m_IntakeShooter);
    return command;
  }

  private void setPivotPositionImpl(double position) {
    this.m_IntakeShooter.setIntakePivotPosition(position);
    this.m_IntakeShooter.setPivotPIDEnabled(true);
  }

  public Command setPivotPosition(double position) {
    InstantCommand command = new InstantCommand(() -> setPivotPositionImpl(position));
    command.addRequirements(this.m_IntakeShooter);
    return command;
  }

  private void resetPivotPositionImpl() {
    this.m_IntakeShooter.resetPivotEncoder();
  }

  public Command resetPivotPosition() {
    InstantCommand command = new InstantCommand(() -> resetPivotPositionImpl());
    command.addRequirements(this.m_IntakeShooter);
    return command;
  }
  
  // public Command stopIntakeShooter(){
  //   Command command = new InstantCommand (() -> m_IntakeShooter.stopIntakeShooter(), m_IntakeShooter);
  //   return command;
  // }

  // public Command stopIntakePivot(){
  //   Command command = new InstantCommand (() -> m_IntakeShooter.stopIntakePivot(), m_IntakeShooter);
  //   return command;
  // }

  // public Command stowIntakePivot(){
  //   Command command = new InstantCommand(() -> m_IntakeShooter.setIntakePivotPosition(IntakeShooterPositions.STOW, IntakeShooterConstants.kIntakePivotff), m_IntakeShooter);
  //   return command;
  // }

  // public Command engageIntakePivot(){
  //   Command command = new InstantCommand(() -> m_IntakeShooter.setIntakePivotPosition(IntakeShooterPositions.ENGAGE, IntakeShooterConstants.kIntakePivotff), m_IntakeShooter);
  //   return command;
  // }
}