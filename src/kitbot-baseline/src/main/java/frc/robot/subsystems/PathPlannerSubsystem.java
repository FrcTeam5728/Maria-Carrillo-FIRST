package frc.robot.subsystems;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PathPlannerSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final DifferentialDriveKinematics kDriveKinematics;

    public PathPlannerSubsystem(DriveSubsystem driveSubsystem, double trackWidth) {
        this.driveSubsystem = driveSubsystem;
        this.kDriveKinematics = new DifferentialDriveKinematics(trackWidth);
    }

    public Pose2d getPose() {
        return driveSubsystem.getPose();
    }

    public void resetPose(Pose2d pose) {
        driveSubsystem.resetOdometry(pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        var wheelSpeeds = driveSubsystem.getWheelSpeeds();
        return new ChassisSpeeds(
            (wheelSpeeds.leftMetersPerSecond + wheelSpeeds.rightMetersPerSecond) / 2,
            0,
            (wheelSpeeds.rightMetersPerSecond - wheelSpeeds.leftMetersPerSecond) / DriveConstants.kTrackWidthMeters
        );
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        var wheelSpeeds = kDriveKinematics.toWheelSpeeds(chassisSpeeds);
        driveSubsystem.tankDriveVolts(
            wheelSpeeds.leftMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond * 12.0,
            wheelSpeeds.rightMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond * 12.0
        );
    }

    public Command followPath(String pathName) {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter
            ),
            kDriveKinematics,
            10 // Max voltage
        );

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            DriveConstants.kMaxSpeedMetersPerSecond,
            DriveConstants.kMaxAccelerationMetersPerSecondSquared
        )
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

        // Load the path from PathPlanner and convert to WPILib trajectory
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        Trajectory trajectory = path.getTrajectory(null, null); // Use default constraints
        
        // Create the command to run the path using Ramsete controller
        return new RamseteCommand(
            trajectory,
            this::getPose,
            new RamseteController(2.0, 0.7), // Tune these values
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter
            ),
            kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            driveSubsystem::tankDriveVolts,
            this
        ).andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
    }

    @Override
    public void periodic() {
        // Update odometry and other periodic tasks
    }
}
