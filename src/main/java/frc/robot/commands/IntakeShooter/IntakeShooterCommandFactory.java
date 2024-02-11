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
import frc.robot.subsystems.utils.IntakeShooterState;
import frc.robot.subsystems.utils.Enums.Position_Enums.IntakeShooterPositions;

// import static frc.robot.Constants.DriveConstants.kMaxAngularSpeed;
// import static frc.robot.Constants.DriveConstants.kMaxSpeedMetersPerSecond;

public class IntakeShooterCommandFactory {

  private final IntakeShooter m_IntakeShooter;

  public IntakeShooterCommandFactory(IntakeShooter m_IntakeShooter) {
    this.m_IntakeShooter = m_IntakeShooter;
  }

  public Command setIntakeShooterPower(double power){
    if (m_IntakeShooter.getState().getPosition() == (IntakeShooterPositions.STOW)) {
      double clampedPower = MathUtil.clamp(power, -0.5, 0.5);
      Command command = new InstantCommand (() -> m_IntakeShooter.setIntakeShooterPower(clampedPower), m_IntakeShooter);
      return command;
    } else {
      System.out.println("Intake Shooter must be in stow position to run the intake shooter");
      return new InstantCommand();
    }
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

  public Command stopShooter(){
    Command command = new InstantCommand (() -> m_IntakeShooter.stopShooter(), m_IntakeShooter);
    return command;
  }

  private void manualPivotImpl(double power) {
    this.m_IntakeShooter.setIntakePivotPower(power);
    this.m_IntakeShooter.setPivotPIDEnabled(false);
    if ((m_IntakeShooter.isStalling())){
      m_IntakeShooter.setZeroPoint();
      m_IntakeShooter.stopPivotMotorPower(); 
    }
  }

  public Command manualPivot(double power) {
    InstantCommand command = new InstantCommand(() -> manualPivotImpl(power));
    command.addRequirements(this.m_IntakeShooter);
    return command;
  }

  private void setPivotPositionImpl(double position) {
    this.m_IntakeShooter.setIntakePivotPosition(position);
    this.m_IntakeShooter.setPivotPIDEnabled(true);
    if ((position == IntakeShooterConstants.kIntakePosition) && (m_IntakeShooter.isStalling())){
      m_IntakeShooter.setZeroPoint();
      m_IntakeShooter.stopPivotMotorPower(); 
    }
  }

  public Command setPivotPosition(IntakeShooterState state) {
    if (state.getPosition() == IntakeShooterPositions.STOW) {
      InstantCommand command = new InstantCommand(() -> setPivotPositionImpl(IntakeShooterConstants.kStowPosition));
      command.addRequirements(this.m_IntakeShooter);
      return command;
    } else if (state.getPosition() == IntakeShooterPositions.INTAKE) {
      InstantCommand command = new InstantCommand(() -> setPivotPositionImpl(IntakeShooterConstants.kIntakePosition));
      command.addRequirements(this.m_IntakeShooter);
      return command;
    } else if (state.getPosition() == IntakeShooterPositions.SHOOT) {
      InstantCommand command = new InstantCommand(() -> setPivotPositionImpl(IntakeShooterConstants.kShootPosition));
      command.addRequirements(this.m_IntakeShooter);
      return command;
    }
    else{
      System.out.println("Invalid intake shooter position");
      return new InstantCommand();
    }
  }

  private void setShooterRPMImpl(double upperRPM, double lowerRPM) {
    this.m_IntakeShooter.setShooterRPM(upperRPM, lowerRPM);
    this.m_IntakeShooter.setShooterPIDEnabled(true);
  }

  public Command setShooterRPM(double upperRPM, double lowerRPM) {
    if (m_IntakeShooter.getState().getPosition() == IntakeShooterPositions.SHOOT) {
      InstantCommand command = new InstantCommand(() -> setShooterRPMImpl(upperRPM, lowerRPM));
      command.addRequirements(this.m_IntakeShooter);
      return command;
    } else {
      System.out.println("Intake Shooter must be in shoot position to run the shooter");
      return new InstantCommand();
    }
  }

  private void resetPivotPositionImpl() {
    this.m_IntakeShooter.resetPivotEncoder();
  }

  public Command resetPivotPosition() {
    InstantCommand command = new InstantCommand(() -> resetPivotPositionImpl());
    command.addRequirements(this.m_IntakeShooter);
    return command;
  }

  public void intakeImpl(){
    this.m_IntakeShooter.setIntakeShooterPower(-0.5);
    this.m_IntakeShooter.setIntakeStowPower(-0.5);
  }

  public Command inititateIntake(){

    if (m_IntakeShooter.getState().getPosition() == IntakeShooterPositions.INTAKE){
      InstantCommand command = new InstantCommand(() -> intakeImpl());
      command.addRequirements(this.m_IntakeShooter);
      return command;
    } else {
      System.out.println("Intake Shooter must be in intake position to run the intake");
      return new InstantCommand();
    }

  }

  public void launchStowMotorShootImpl(){
    this.m_IntakeShooter.setIntakeStowPower(1.0);
  }

  public Command launchStowMotorShoot(){
    if (m_IntakeShooter.getState().getPosition() == IntakeShooterPositions.SHOOT){
      InstantCommand command = new InstantCommand(() -> launchStowMotorShootImpl());
      command.addRequirements(this.m_IntakeShooter);
      return command;
    } else {
      System.out.println("Intake Shooter must be in shoot position to run the stow motor");
      return new InstantCommand();
    }

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