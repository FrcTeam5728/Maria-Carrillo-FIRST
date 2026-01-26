adf

package frc.robot.subsystems.pose_estimation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * A simple pose estimator that uses only odometry from the swerve drive.
 * This provides a basic estimation of the robot's pose based on wheel encoder
 * and gyro data, without any vision correction.
 */
public class OdometryPoseEstimator {
    private final SwerveSubsystem swerveSubsystem;
    private double lastUpdateTime = 0;
    private Pose2d lastPose = new Pose2d();
    private double lastTimestamp = 0;

    /**
     * Creates a new OdometryPoseEstimator.
     *
     * @param swerveSubsystem The swerve drive subsystem to get odometry data from.
     */
    public OdometryPoseEstimator(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        resetPose(new Pose2d());
    }

    /**
     * Updates the pose estimation with the latest odometry data.
     * This should be called periodically, typically in the robot's periodic method.
     */
    public void update() {
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - lastUpdateTime;
        
        // Get the current pose from the swerve drive's odometry
        lastPose = swerveSubsystem.getPose();
        lastUpdateTime = currentTime;
    }

    /**
     * Gets the estimated pose of the robot.
     *
     * @return The estimated pose of the robot.
     */
    public Pose2d getEstimatedPose() {
        return lastPose;
    }

    /**
     * Resets the pose estimator to the specified pose.
     * This should be called when the robot's position is known (e.g., at the start of a match).
     *
     * @param pose The pose to reset to.
     */
    public void resetPose(Pose2d pose) {
        swerveSubsystem.resetOdometry(pose);
        lastPose = pose;
        lastUpdateTime = Timer.getFPGATimestamp();
    }

    /**
     * Gets the estimated velocity of the robot.
     *
     * @return The estimated velocity in meters per second.
     */
    public double getVelocity() {
        // Calculate velocity based on change in position over time
        if (lastTimestamp == 0) {
            lastTimestamp = Timer.getFPGATimestamp();
            return 0;
        }
        
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - lastTimestamp;
        lastTimestamp = currentTime;
        
        if (deltaTime <= 0) {
            return 0;
        }
        
        // This is a simple implementation that calculates the magnitude of the velocity vector
        // For more accurate velocity estimation, you might want to use the chassis speeds
        // from the swerve drive's odometry
        Pose2d currentPose = swerveSubsystem.getPose();
        double dx = currentPose.getX() - lastPose.getX();
        double dy = currentPose.getY() - lastPose.getY();
        
        return Math.hypot(dx, dy) / deltaTime;
    }

    /**
     * Gets the estimated angular velocity of the robot.
     *
     * @return The estimated angular velocity in radians per second.
     */
    public double getAngularVelocity() {
        // Get the angular velocity from the gyro if available
        // This is a placeholder - you'll need to implement this based on your gyro
        return 0; // Replace with actual implementation
    }
}
