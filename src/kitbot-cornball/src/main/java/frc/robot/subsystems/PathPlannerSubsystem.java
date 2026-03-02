package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * PathPlanner subsystem for following autonomous paths.
 * Uses PathPlanner 2026.1.2 with modern AutoBuilder configuration.
 */
public class PathPlannerSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private final DifferentialDriveKinematics kDriveKinematics;
    private boolean autoBuilderConfigured = false;

    public PathPlannerSubsystem(DriveSubsystem driveSubsystem, double trackWidth) {
        this.driveSubsystem = driveSubsystem;
        this.kDriveKinematics = new DifferentialDriveKinematics(trackWidth);
        
        // Configure AutoBuilder for differential drive
        configureAutoBuilder();
    }

    /**
     * Configures AutoBuilder for differential drive robots.
     * This should be called once in the constructor.
     */
    private void configureAutoBuilder() {
        if (autoBuilderConfigured) return;
        
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            
            AutoBuilder.configure(
                this::getPose,                    // Robot pose supplier
                this::resetPose,                  // Pose resetter
                this::getRobotRelativeSpeeds,     // ChassisSpeeds supplier
                this::driveRobotRelative,         // ChassisSpeeds consumer
                new PPLTVController(0.02),        // Path following controller for differential drive
                config,                           // Robot configuration
                () -> false,                      // Alliance flip supplier (always blue for now)
                this                              // Subsystem requirements
            );
            
            autoBuilderConfigured = true;
            System.out.println("PathPlanner AutoBuilder configured successfully");
        } catch (Exception e) {
            System.err.println("Failed to configure AutoBuilder: " + e.getMessage());
        }
    }

    /**
     * Creates a command to follow a path with the given name.
     * 
     * @param pathName The name of the path file (without .path extension)
     * @return Command that will follow the path
     */
    public Command followPath(String pathName) {
        if (!autoBuilderConfigured) {
            System.err.println("AutoBuilder not configured, cannot follow path: " + pathName);
            return null;
        }
        
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
        } catch (Exception e) {
            System.err.println("Failed to create path following command for: " + pathName);
            System.err.println("Error: " + e.getMessage());
            return null;
        }
    }

    /**
     * Gets the current robot pose from the drive subsystem.
     * Required by AutoBuilder.
     */
    private Pose2d getPose() {
        return driveSubsystem.getPose();
    }

    /**
     * Resets the robot pose in the drive subsystem.
     * Required by AutoBuilder.
     */
    private void resetPose(Pose2d pose) {
        driveSubsystem.resetOdometry(pose);
    }

    /**
     * Gets the current robot-relative chassis speeds.
     * Required by AutoBuilder for differential drive.
     */
    private ChassisSpeeds getRobotRelativeSpeeds() {
        // Convert wheel speeds to chassis speeds
        var wheelSpeeds = driveSubsystem.getWheelSpeeds();
        return kDriveKinematics.toChassisSpeeds(wheelSpeeds);
    }

    /**
     * Drives the robot with robot-relative chassis speeds.
     * Required by AutoBuilder.
     */
    private void driveRobotRelative(ChassisSpeeds speeds) {
        // Convert chassis speeds to wheel speeds
        var wheelSpeeds = kDriveKinematics.toWheelSpeeds(speeds);
        
        // Convert to voltages (simplified - you may want to add feedforward)
        double leftVoltage = wheelSpeeds.leftMetersPerSecond * 0.1; // Scale factor
        double rightVoltage = wheelSpeeds.rightMetersPerSecond * 0.1; // Scale factor
        
        driveSubsystem.tankDriveVolts(leftVoltage, rightVoltage);
    }

    @Override
    public void periodic() {
        // Update odometry and other periodic tasks if needed
    }
}
