package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PathPlannerSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final DifferentialDriveKinematics kinematics;

    public PathPlannerSubsystem(DriveSubsystem driveSubsystem, double trackWidth) {
        this.driveSubsystem = driveSubsystem;
        this.kinematics = new DifferentialDriveKinematics(trackWidth);

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation PID constants
                new PIDConstants(0.5, 0, 0), // Rotation PID constants
                DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed
                Math.sqrt(2) * (DriveConstants.kTrackWidthMeters / 2.0), // Drive base radius
                new ReplanningConfig() // Default path replanning config
            ),
            () -> {
                // Boolean supplier that controls path mirroring for the red alliance
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    public Pose2d getPose() {
        return driveSubsystem.getPose();
    }

    public void resetPose(Pose2d pose) {
        driveSubsystem.resetOdometry(pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        var wheelSpeeds = driveSubsystem.getWheelSpeeds();
        return kinematics.toChassisSpeeds(
            wheelSpeeds.leftMetersPerSecond,
            wheelSpeeds.rightMetersPerSecond
        );
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        driveSubsystem.tankDriveVolts(
            wheelSpeeds.leftMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond * 12.0,
            wheelSpeeds.rightMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond * 12.0
        );
    }

    public Command followPath(String pathName) {
        // Load the path we want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        
        // Create a path following command using AutoBuilder
        return new PathPlannerAuto(pathName);
    }

    @Override
    public void periodic() {
        // Update odometry and other periodic tasks
    }
}
