
package frc.robot.commands.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Optional;
import java.util.ArrayList;
import java.util.List;

public class NeoTrajectory {

    
    private static Pose2d lastEnd = new Pose2d(0, 0, new Rotation2d(0));
    private List<List<Pose2d>> waypointGroups = new ArrayList<>();
    private List<TrajectoryConfig> configs = new ArrayList<>();
    private List<Pose2d> currentWaypoints = new ArrayList<>();
    private boolean isReversed;

    public NeoTrajectory() {
        addNewConfig();
    }

    public NeoTrajectory(TrajectoryConfig config) {
        configs.add(config);
    }

    public NeoTrajectory(double x, double y, double rot) {
        this();
        addWaypoint(x, y, rot);
    }

    private void addNewConfig() {
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(isReversed);
        configs.add(config);
    }

    public void addWaypoint(double x, double y, double rot) {
        Rotation2d rotation = new Rotation2d(Math.toRadians(rot));
        addWaypoint(x, y, rotation);
    }


    public void addWaypoint(double x, double y) {

        Rotation2d rotation;
        
        if (!currentWaypoints.isEmpty()) {
            Pose2d lastWaypoint = currentWaypoints.get(currentWaypoints.size() - 1);
            rotation = new Rotation2d(x - lastWaypoint.getTranslation().getX(), y - lastWaypoint.getTranslation().getY());
            Translation2d currentWayPoint = new Translation2d(x, y);
            Translation2d delta = currentWayPoint.minus(lastWaypoint.getTranslation()).rotateBy(rotation.unaryMinus());
            boolean newIsReversed = delta.getX() < 0;
            if (newIsReversed != isReversed) {
                isReversed = newIsReversed;
                addNewConfig();
            }
        } else {
            rotation = new Rotation2d(0);
        }

        addWaypoint(x, y, rotation);
        waypointGroups.add(currentWaypoints); // Update waypointGroups every time a waypoint is added
        currentWaypoints = new ArrayList<>(); // Clear currentWaypoints for the next group of waypoints
        System.out.println("Current waypoints size: " + currentWaypoints.size());
    }

    private void addWaypoint(double x, double y, Rotation2d rotation) {
        Pose2d waypoint = new Pose2d(x, y, rotation);
        currentWaypoints.add(waypoint);
        lastEnd = waypoint;
    }



    public List<Trajectory> generateTrajectories() {
        if (!currentWaypoints.isEmpty()) {
            waypointGroups.add(new ArrayList<>(currentWaypoints)); // Add currentWaypoints to waypointGroups
            currentWaypoints.clear(); // Clear currentWaypoints for the next group of waypoints
        }
        System.out.println("Waypoint groups size: " + waypointGroups.size()); // Debug statement
        List<Trajectory> trajectories = new ArrayList<>();
        for (int i = 0; i < waypointGroups.size(); i++) {
            trajectories.add(TrajectoryGenerator.generateTrajectory(waypointGroups.get(i), configs.get(i)));
        }
        return trajectories;
    }

    // public Pose2d getStartingWaypointInitialPose() {
    //     System.out.println("Getting initial pose. Waypoint groups size: " + waypointGroups.size()); // Debug statement
    //     if (waypointGroups.isEmpty()) {
    //         System.out.println("Warning: waypointGroups is empty. Returning default Pose2d object.");
    //         return new Pose2d(0, 0, new Rotation2d(0));
    //     }
    //     return waypointGroups.get(0).get(0);
    // }

    public Pose2d getStartingWaypointInitialPose() {
        System.out.println("Getting initial pose. Waypoint groups size: " + waypointGroups.size()); // Debug statement
        if (waypointGroups.isEmpty()) {
            generateTrajectories(); // Generate trajectories to update waypointGroups
        }
        return waypointGroups.get(0).get(0);
    }

    public SequentialCommandGroup generateCommand(SwerveDrive m_robotDrive) {
        List<Trajectory> trajectories = this.generateTrajectories();
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        for (Trajectory trajectory : trajectories) {
            SwerveControllerCommand command = new SwerveControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                // Position controllers
                new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
                new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController),
                new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController,
                    AutoConstants.kThetaControllerConstraints),
                () -> lastEnd.getRotation(),
                m_robotDrive::setModuleStates,
                m_robotDrive
            );
            commandGroup.addCommands(command);
        }
        return commandGroup;
    }
}