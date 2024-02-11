// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeShooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

//import com.kauailabs.navx.frc.AHRS;
//import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.utils.MotorControlGroup;
import frc.robot.subsystems.utils.SwerveUtils;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;


public class IntakeShooter extends SubsystemBase {
    private static final int PIVOT_AVERAGE_WINDOW = 10;
    private CANSparkMax pivotMotor;
    private double targetUpperShooterRPM;
    private double targetLowerShooterRPM;

    private CANSparkMax upperShooterMotor;
    private CANSparkMax lowerShooterMotor;
    private CANSparkMax stowMotor;
    private PIDController pivotController;
    private PIDController shooterController;

    private double[] lastPivotCurrents;
    private int currentPivotCurrentIdx;

    private boolean pivotPIDEnabled;

    public IntakeShooter() {
        pivotMotor = new CANSparkMax(IntakeShooterConstants.kIntakePivotMotor, MotorType.kBrushless);
        lowerShooterMotor = new CANSparkMax(IntakeShooterConstants.kIntakeShooterLowerMotor, MotorType.kBrushless);
        upperShooterMotor = new CANSparkMax(IntakeShooterConstants.kIntakeShooterUpperMotor, MotorType.kBrushless);
        stowMotor = new CANSparkMax(IntakeShooterConstants.kIntakeShooterStowMotor, MotorType.kBrushless);

        upperShooterMotor.setInverted(false);
        lowerShooterMotor.setInverted(false);

        pivotController = new PIDController(
            IntakeShooterConstants.kIntakePivotP,
            IntakeShooterConstants.kIntakePivotI,
            IntakeShooterConstants.kIntakePivotD
        );
        pivotController.setIntegratorRange(0.0, 0.05);
        this.pivotPIDEnabled = false;

        lastPivotCurrents = new double[PIVOT_AVERAGE_WINDOW];
        for(int i = 0; i < PIVOT_AVERAGE_WINDOW; i++) {
            lastPivotCurrents[i] = 0.0;
        }
        currentPivotCurrentIdx = 0;
    }

    public void setIntakeShooterPower(double power) {
        lowerShooterMotor.set(power);
        upperShooterMotor.set(power);
    }

    public void setIntakeStowPower(double power) {
        stowMotor.set(power);
    }

    public void intake() {
        lowerShooterMotor.set(-0.5);
        upperShooterMotor.set(-0.5);
        stowMotor.set(-0.5);
    }

    public void outtake() {
        lowerShooterMotor.set(0.5);
        upperShooterMotor.set(0.5);
        stowMotor.set(0.5);
    }

    public void spinUp() {
        lowerShooterMotor.set(1.0);
        upperShooterMotor.set(1.0);
    }

    public void shoot() {
        // add check for shooter rpm
        if (upperShooterMotor.getEncoder().getVelocity() >= targetUpperShooterRPM && lowerShooterMotor.getEncoder().getVelocity() >= targetLowerShooterRPM) {
            lowerShooterMotor.set(1.0);
            upperShooterMotor.set(1.0);
        }
        
        stowMotor.set(1.0);
    }

    public void setIntakePivotPower(double power) {
        pivotMotor.set(power);
    }

    public void setIntakePivotPosition(double position) {
        pivotController.setSetpoint(position);
    }

    public void setPivotPIDEnabled(boolean enabled) {
        this.pivotPIDEnabled = enabled;
    }

    public void resetPivotEncoder() {
        pivotMotor.getEncoder().setPosition(0);
    }

    // public void setIntakePivotPosition(IntakeShooterPositions position , double ff) {
        
    //     if (position == IntakeShooterPositions.ENGAGE) {
    //         intakePivotMotor.setPosition(IntakeShooterConstants.kIntakePivotEngagePosition, ff);
    //     } else if (position == IntakeShooterPositions.STOW) {
    //         intakePivotMotor.setPosition(IntakeShooterConstants.kIntakePivotStowPosition, ff);
    //     }

    // }

    // public void setIntakePivotBrakeMode(boolean brakeMode) {
    //     intakePivotMotor.setDefaultBrakeMode(brakeMode);
    // }

    // public void setIntakeShooterBrakeMode(boolean brakeMode) {
    //     intakeShooterMotors.setDefaultBrakeMode(brakeMode);
    // }

    private void updatePivotAverage() {
        lastPivotCurrents[currentPivotCurrentIdx] = pivotMotor.getOutputCurrent();
        currentPivotCurrentIdx++;
        currentPivotCurrentIdx %= PIVOT_AVERAGE_WINDOW;
    }

    public double getAveragePivotCurrent() {
        double sum = 0.0;
        for(int i = 0; i < PIVOT_AVERAGE_WINDOW; i++) {
            sum += lastPivotCurrents[i];
        }
        return sum / PIVOT_AVERAGE_WINDOW;
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Pivot Encoder", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Upper Shooter RPM", upperShooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Shooter RPM", lowerShooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Pivot Current", getAveragePivotCurrent());
        SmartDashboard.putBoolean("Pivot Stall", getAveragePivotCurrent() >= 18);

        updatePivotAverage();

        if(this.pivotPIDEnabled) {
            double pivotControl = pivotController.calculate(pivotMotor.getEncoder().getPosition());
            if(pivotControl > 0.35) {
                pivotControl = 0.35;
            }
            else if(pivotControl < -0.35) {
                pivotControl = -0.35;
            }
            pivotMotor.set(pivotControl);
        }
    }
}