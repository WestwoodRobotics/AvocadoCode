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
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.utils.MotorControlGroup;
import frc.robot.subsystems.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;


public class IntakeShooter extends SubsystemBase {

    private MotorControlGroup intakeShooterMotors;

    private MotorControlGroup intakePivotMotor;

    public IntakeShooter() {
        intakeShooterMotors = new MotorControlGroup(new CANSparkMax(IntakeShooterConstants.kIntakeShooterMotor1, MotorType.kBrushless), new CANSparkMax(IntakeShooterConstants.kIntakeShooterMotor2, MotorType.kBrushless));
        intakePivotMotor = new MotorControlGroup(new CANSparkMax(IntakeShooterConstants.kIntakePivotMotor, MotorType.kBrushless));
        intakePivotMotor.setDefaultBrakeMode(true);
        intakeShooterMotors.setDefaultBrakeMode(true);
    }

    public void setIntakeShooterPower(double power) {
        intakeShooterMotors.setPower(power);
    }

    public void setIntakePivotPower(double power) {
        intakePivotMotor.setPower(power);
    }

    public void stopIntakeShooter() {
        intakeShooterMotors.setPower(0);
    }

    public void stopIntakePivot() {
        intakePivotMotor.setPower(0);
    }

    public void setIntakePivotPosition(double position) {
        intakePivotMotor.setPosition((int)position, 0);
    }

    public void setIntakePivotPosition(double position , double ff) {
        intakePivotMotor.setPosition(position, ff);
    }

    public void setIntakePivotBrakeMode(boolean brakeMode) {
        intakePivotMotor.setDefaultBrakeMode(brakeMode);
    }

    public void setIntakeShooterBrakeMode(boolean brakeMode) {
        intakeShooterMotors.setDefaultBrakeMode(brakeMode);
    }










}