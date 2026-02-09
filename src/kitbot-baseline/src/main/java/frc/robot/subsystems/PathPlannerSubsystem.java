package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PathPlannerSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final DifferentialDriveKinematics kinematics;

    /**
     * Lightweight stub of a path-following subsystem. The original project
     * referenced PathPlanner (optional third-party dependency). To keep the
     * project compiling without that library, we provide a minimal wrapper here.
     * Teams that want full PathPlanner support should restore the original
     * implementation and add the PathPlanner dependency to build.gradle.
     */
    public PathPlannerSubsystem(DriveSubsystem driveSubsystem, double trackWidth) {
        this.driveSubsystem = driveSubsystem;
        this.kinematics = new DifferentialDriveKinematics(trackWidth);
    }

    public Pose2d getPose() {
        return driveSubsystem.getPose();
    }

    public void resetPose(Pose2d pose) {
        driveSubsystem.resetOdometry(pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        var wheelSpeeds = driveSubsystem.getWheelSpeeds();
        // Use the kinematics helper that accepts a DifferentialDriveWheelSpeeds
        return kinematics.toChassisSpeeds(wheelSpeeds);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        var wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        // Map wheel speeds to voltages. Use configured max speed if available.
        double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond > 0
            ? DriveConstants.kMaxSpeedMetersPerSecond
            : 1.0;

        driveSubsystem.tankDriveVolts(
            wheelSpeeds.leftMetersPerSecond / maxSpeed * 12.0,
            wheelSpeeds.rightMetersPerSecond / maxSpeed * 12.0
        );
    }

    /**
     * Placeholder followPath. Returns null for now; teams should replace this
     * with a real PathPlanner command if they add that dependency.
     */
    public Command followPath(String pathName) {
        return null;
    }

    @Override
    public void periodic() {
        // Update odometry and periodic tasks if required
    }
}
