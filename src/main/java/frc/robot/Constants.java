// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkBase.IdleMode;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.utils.IntakeShooterState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = Math.PI/3; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));



    // Angular offsets of the modules relative to the chassis in radians
    // These values are the angle offset of the wheels when the robot is facing forwards (Absolute Encoders)
    //DO NOT CHANGE THESE VALUES UNLESS YOU KNOW WHAT YOU'RE DOING!!
    public static final double kFrontLeftChassisAngularOffset = 3*Math.PI/2 - 0.05;
    public static final double kFrontRightChassisAngularOffset = Math.PI;
    public static final double kRearLeftChassisAngularOffset = 0;
    public static final double kRearRightChassisAngularOffset =  3*Math.PI/2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 15;
    public static final int kRearLeftDrivingCanId = 17;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearRightDrivingCanId = 11;

    public static final int kFrontLeftTurningCanId = 14;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 10;

    public static final boolean kGyroReversed = false;

    public static final double slowModeMultiplier = 0.25;
  }

  public static final class PortConstants{
        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 15;
        public static final int kRearLeftDrivingCanId = 17;
        public static final int kFrontRightDrivingCanId = 13;
        public static final int kRearRightDrivingCanId = 11;
    
        public static final int kFrontLeftTurningCanId = 14;
        public static final int kRearLeftTurningCanId = 16;
        public static final int kFrontRightTurningCanId = 12;
        public static final int kRearRightTurningCanId = 10;
        

        public static final int kElevatorMotor1Port = 20; 
        public static final int kElevatorMotor2Port = 21;
        public static final int kIntakeMotorPort = 22;

        public static final int kLEDPort = 0;
        public static final int kLEDLength = 5;
        public static final int kLimitSwitchPort = 0;

  }

  public static final class TransportConstants{

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians


    // These PID Gains have been tested
    public static final double kDrivingP = 0.1;
    public static final double kDrivingI = 0; 
    public static final double kDrivingD = 0.002; 
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps; 

    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.2;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    public static final int testCANId = 0; //TODO: Change this to the CAN ID of the motor you want to test
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int phoenixGyroPort = 0; //TODO: Check if this is correct
    public static final double kDriveDeadband = 0.2;



  }

  public static final class AutoConstants {

    //These constants need to tuned when setting up Auton Paths
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;


    //These control x, y, and theta corrections when doing path following, 
    //basically like a joystick input to correct for misalignment, 
    //Units are m/s per meter of offset or rad/s per radian of offset

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 2;
    
    public static final double kIXController = 0;
    public static final double kIYController = 0;
    public static final double kIThetaController = 0;

    public static final double kDXController = 0;
    public static final double kDYController = 0;
    public static final double kDThetaController = 0;



    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
  public static final class ElevatorConstants {
    public static final int kElevatorMotor = 20;

    public static double kP = 0; //TODO: Change this value
    public static double kI = 0; //TODO: Change this value
    public static double kD = 0; //TODO: Change this value
    public static double ff = 0; //TODO: Change this value

    public static int elevatorClimbInit = 0; //TODO: Change this value
    public static int elevatorClimbHome = 0; //TODO: Change this value

    
  }
  public static final class IntakeShooterConstants {
    public static final int kIntakeShooterLowerMotor = 22; //TODO: Change this value
    public static final int kIntakeShooterUpperMotor = 21; //TODO: Change this value
    public static final int kIntakeShooterStowMotor = 20; //TODO: Change this value
    public static final int kIntakePivotMotor = 23; //TODO: Change this value

    
    public static final double kStowPosition = 0.568648;
    public static final double kIntakePosition = -0.963421; //TODO: Change this value
    public static final double kShootSpeakerBasePosition = 1.968733; //TODO: Change this value
    public static final double kShootNoteLinePosition = 0; //TODO: Change this value
    
    public static double kIntakePivotStowPosition = 0;
    public static double kIntakePivotEngagePosition = 0;

    public static double kIntakePivotP = 1;
    public static double kIntakePivotI = 0;
    public static double kIntakePivotD = 0;
    public static double kIntakePivotff = 0;

    public static double kIntakeUpperShooterP = 0.08;
    public static double kIntakeUpperShooterI = 0.00;
    public static double kIntakeUpperShooterD = 0;
    public static double kIntakeUpperShooterff = 0.9;

    // public static double kIntakeUpperShooterP = 0;
    // public static double kIntakeUpperShooterI = 0;
    // public static double kIntakeUpperShooterD = 0;
    // public static double kIntakeUpperShooterff = 0;

    public static double kIntakeLowerShooterP = 0.00035;
    public static double kIntakeLowerShooterI = 0.00000;
    public static double kIntakeLowerShooterD = 0.00000;
    public static double kIntakeLowerShooterff = 0.9;
  }

  public static final class ArmConstants{
    public static final int kArmMotor = 21;

    public static double kP = 0; //TODO: Change this value
    public static double kI = 0; //TODO: Change this value
    public static double kD = 0; //TODO: Change this value

    public static int arm_cube_pickup = 70; //also arm cone high and mid cube outtake
    public static int arm_cone_pickup = 100; //also arm cone mid outtake
    public static int arm_cube_outtake = 0; //refers to high and low outtake
  }



}