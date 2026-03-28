// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Command that follows a PathPlanner path using differential drive.
 */
public class FollowPathPlannerPath extends Command {
  private final PathPlannerPath m_path;
  private final DriveSubsystem m_drive;
  private final Supplier<Pose2d> m_poseSupplier;
  private final boolean m_isFirstPath;

  private com.pathplanner.lib.commands.FollowPathCommand m_followCommand;

  /**
   * Creates a new FollowPathPlannerPath command for differential drive.
   * 
   * @param path The PathPlanner path to follow
   * @param drive The differential drive subsystem
   * @param poseSupplier Supplier for the current robot pose
   * @param isFirstPath Whether this is the first path in the auto sequence (for odometry reset)
   */
  public FollowPathPlannerPath(
      PathPlannerPath path,
      DriveSubsystem drive,
      Supplier<Pose2d> poseSupplier,
      boolean isFirstPath) {
    m_path = path;
    m_drive = drive;
    m_poseSupplier = poseSupplier;
    m_isFirstPath = isFirstPath;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Configure the path follower for differential drive
    com.pathplanner.lib.config.PIDConstants translationPID = new PIDConstants(5.0, 0.0, 0.0);
    com.pathplanner.lib.config.PIDConstants rotationPID = new PIDConstants(5.0, 0.0, 0.0);
    double maxSpeed = 3.0; // Max speed in m/s (adjust based on your robot)
    
    // Create the path following command for differential drive
    m_followCommand = new com.pathplanner.lib.commands.FollowPathCommand(
        m_path,
        m_poseSupplier,
        speeds -> {
          // Convert chassis speeds to differential drive voltages
          double leftSpeed = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * 0.381;
          double rightSpeed = speeds.vxMetersPerSecond + speeds.omegaRadiansPerSecond * 0.381;
          
          // Apply to drive subsystem using arcade drive
          double forward = speeds.vxMetersPerSecond / maxSpeed;
          double rotation = speeds.omegaRadiansPerSecond / (maxSpeed / 0.381);
          
          m_drive.driveArcade(() -> forward, () -> rotation);
        },
        translationPID,
        rotationPID,
        maxSpeed,
        m_drive
    );

    // Reset odometry if this is the first path
    if (m_isFirstPath) {
      m_drive.resetOdometry(m_path.getStartingDifferentialPose());
    }

    // Initialize the follow command
    m_followCommand.schedule();
  }

  @Override
  public void execute() {
    // The follow command handles execution
  }

  @Override
  public void end(boolean interrupted) {
    if (m_followCommand != null) {
      m_followCommand.cancel();
    }
    m_drive.driveArcade(() -> 0, () -> 0);
  }

  @Override
  public boolean isFinished() {
    return m_followCommand != null && m_followCommand.isFinished();
  }

  /**
   * Static method to configure AutoBuilder for differential drive.
   * Call this once in your robot initialization.
   * 
   * @param drive The differential drive subsystem
   */
  public static void configureAutoBuilder(DriveSubsystem drive) {
    com.pathplanner.lib.config.RobotConfig config = new com.pathplanner.lib.config.RobotConfig(
        3.0, // Max speed m/s
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID
        new ReplanningConfig() // Default replanning config
    );

    AutoBuilder.configure(
        drive::getPose, // Pose supplier
        drive::resetOdometry, // Pose resetter (for first path)
        speeds -> {
          // Convert chassis speeds to differential drive
          double leftSpeed = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * 0.381;
          double rightSpeed = speeds.vxMetersPerSecond + speeds.omegaRadiansPerSecond * 0.381;
          
          // Apply to drive subsystem
          double forward = speeds.vxMetersPerSecond / 3.0;
          double rotation = speeds.omegaRadiansPerSecond / (3.0 / 0.381);
          
          drive.driveArcade(() -> forward, () -> rotation);
        },
        config, // Path follower config
        () -> false, // Should flip path (always false for this use case)
        drive // Reference to drive subsystem
    );
  }

  /**
   * Creates a command that follows a named PathPlanner path.
   * 
   * @param pathName The name of the path (without .path extension)
   * @param drive The differential drive subsystem
   * @param isFirstPath Whether this is the first path in auto
   * @return Command that follows the path
   */
  public static Command followNamedPath(String pathName, DriveSubsystem drive, boolean isFirstPath) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return new FollowPathPlannerPath(path, drive, drive::getPose, isFirstPath);
  }

  /**
   * Creates a PathPlanner autonomous command from a named auto.
   * 
   * @param autoName The name of the auto (from autos.json)
   * @return Command that runs the full autonomous routine
   */
  public static Command createAutoCommand(String autoName) {
    return new PathPlannerAuto(autoName);
  }
}
