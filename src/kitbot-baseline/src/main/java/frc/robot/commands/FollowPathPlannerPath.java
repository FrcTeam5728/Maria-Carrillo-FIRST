// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Command that follows a PathPlanner path using swerve drive.
 */
public class FollowPathPlannerPath extends Command {
  private final PathPlannerPath m_path;
  private final SwerveDriveSubsystem m_drive;
  private final Supplier<Pose2d> m_poseSupplier;
  private final boolean m_isFirstPath;

  private com.pathplanner.lib.commands.FollowPathHolonomic m_followCommand;

  /**
   * Creates a new FollowPathPlannerPath command.
   * 
   * @param path The PathPlanner path to follow
   * @param drive The swerve drive subsystem
   * @param poseSupplier Supplier for the current robot pose
   * @param isFirstPath Whether this is the first path in the auto sequence (for odometry reset)
   */
  public FollowPathPlannerPath(
      PathPlannerPath path,
      SwerveDriveSubsystem drive,
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
    // Configure the path follower
    HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID  
        4.5, // Max module speed (m/s)
        0.4, // Drive base radius (m)
        new ReplanningConfig() // Default replanning config
    );

    // Create the path following command
    m_followCommand = new com.pathplanner.lib.commands.FollowPathHolonomic(
        m_path,
        m_poseSupplier,
        m_drive::getHeading, // Robot heading supplier
        m_drive::drive, // Chassis speeds consumer (robot-relative)
        config,
        () -> false, // Should flip path (always false for this use case)
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
    m_drive.stop();
  }

  @Override
  public boolean isFinished() {
    return m_followCommand != null && m_followCommand.isFinished();
  }

  /**
   * Static method to configure AutoBuilder for swerve drive.
   * Call this once in your robot initialization.
   * 
   * @param drive The swerve drive subsystem
   */
  public static void configureAutoBuilder(SwerveDriveSubsystem drive) {
    HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID
        4.5, // Max module speed (m/s) - should match your drive's max speed
        0.4, // Drive base radius (m)
        new ReplanningConfig() // Default replanning config
    );

    AutoBuilder.configureHolonomic(
        drive::getPose, // Pose supplier
        drive::resetOdometry, // Pose resetter (for first path)
        drive::getChassisSpeeds, // Chassis speeds supplier
        drive::driveRobotRelative, // Chassis speeds consumer (robot-relative)
        config, // Path follower config
        () -> false, // Should flip path
        drive // Reference to drive subsystem
    );
  }

  /**
   * Creates a command that follows a named PathPlanner path.
   * 
   * @param pathName The name of the path (without .path extension)
   * @param drive The swerve drive subsystem
   * @param isFirstPath Whether this is the first path in auto
   * @return Command that follows the path
   */
  public static Command followNamedPath(String pathName, SwerveDriveSubsystem drive, boolean isFirstPath) {
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
