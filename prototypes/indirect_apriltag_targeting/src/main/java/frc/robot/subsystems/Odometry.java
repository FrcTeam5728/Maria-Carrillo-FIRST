package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Odometry subsystem that combines IMU and wheel encoder data for position estimation,
 * with AprilTag vision measurements for correction.
 */
public class Odometry extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final LimelightSubsystem limelight;
    private final Supplier<SwerveModulePosition[]> modulePositions;
    private final Supplier<Rotation2d> gyroAngle;
    private SwerveDriveKinematics kinematics;
    
    // Standard deviations of the vision measurements. Increase these numbers to trust sensor readings less.
    private static final double[] VISION_MEASUREMENT_STD_DEVS = {0.5, 0.5, Units.degreesToRadians(30)};
    
    // Standard deviations for the model. Increase these numbers to trust the model's state estimates less.
    private static final double[] STATE_STD_DEVS = {0.1, 0.1, Units.degreesToRadians(5)};

    /**
     * Creates a new Odometry subsystem.
     * 
     * @param kinematics The swerve drive kinematics
     * @param gyroAngle Supplier for the current gyro angle
     * @param modulePositions Supplier for the current module positions
     * @param initialPose The initial pose of the robot
     */
    public Odometry(SwerveDriveKinematics kinematics, Supplier<Rotation2d> gyroAngle,
                   Supplier<SwerveModulePosition[]> modulePositions, Pose2d initialPose) {
        this.kinematics = kinematics;
        this.gyroAngle = gyroAngle;
        this.modulePositions = modulePositions;
        this.limelight = new LimelightSubsystem();
    

        // Use the kinematics parameter directly in the pose estimator
        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,  // Use the parameter directly
            gyroAngle.get(),
            modulePositions.get(),
            initialPose,
            VecBuilder.fill(STATE_STD_DEVS[0], STATE_STD_DEVS[1], STATE_STD_DEVS[2]),
            VecBuilder.fill(VISION_MEASUREMENT_STD_DEVS[0], VISION_MEASUREMENT_STD_DEVS[1], VISION_MEASUREMENT_STD_DEVS[2])
        );
    }

    @Override
    public void periodic() {
        // Update the pose estimator with the latest gyro and module position readings
        poseEstimator.update(gyroAngle.get(), modulePositions.get());
        
        // Process vision measurements if available
        updateFromVision();
    }
    
    /**
     * Updates the pose estimator with vision measurements if a valid target is detected.
     */
    private void updateFromVision() {
        if (!limelight.hasTarget()) {
            return;
        }
        
        // Get the timestamp of the vision measurement
        // Limelight doesn't provide latency directly, so we'll use the current time
        // and rely on the vision measurement's timestamp from NetworkTables if needed
        double timestamp = Timer.getFPGATimestamp();
        
        // Get the robot's current pose from the vision system
        // This is a simplified example - you'll need to implement getRobotPoseFromVision()
        // to convert the limelight data to a Pose2d
        Optional<Pose2d> visionPose = getRobotPoseFromVision();
        
        if (visionPose.isPresent()) {
            // Add the vision measurement to the pose estimator
            poseEstimator.addVisionMeasurement(visionPose.get(), timestamp);
        }
    }
    
    /**
     * Converts vision data from the limelight to a robot pose.
     * 
     * @return The estimated robot pose from vision, if available
     */
    public Optional<Pose2d> getRobotPoseFromVision() {
        if (!limelight.hasTarget()) {
            return Optional.empty();
        }
        
        // Get the ID of the detected AprilTag
        int tagId = limelight.getTid();
        
        // Get the field-relative position of the AprilTag
        // You'll need to implement this based on your field layout
        Optional<Pose2d> tagPose = getTagPose(tagId);
        
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }
        
        // Get the robot's offset from the tag
        double tx = limelight.getTx();
        double ty = limelight.getTy();
        double distance = calculateDistanceToTag(ty);
        
        // Calculate the robot's pose based on the tag's pose and the offset
        Rotation2d robotAngle = tagPose.get().getRotation().plus(Rotation2d.fromDegrees(tx));
        Translation2d robotTranslation = tagPose.get().getTranslation()
            .plus(new Translation2d(distance, robotAngle));
            
        return Optional.of(new Pose2d(robotTranslation, robotAngle));
    }
    
    /**
     * Gets the field-relative pose of a specific AprilTag.
     * 
     * @param tagId The ID of the AprilTag
     * @return The pose of the AprilTag, if known
     */
    private Optional<Pose2d> getTagPose(int tagId) {
        // TODO: Implement this based on your field layout
        // Return the known field-relative position of the AprilTag
        // Return Optional.empty() if the tag ID is not recognized
        return Optional.empty();
    }
    
    /**
     * Calculates the distance to an AprilTag based on its vertical angle.
     * 
     * @param ty The vertical angle to the tag in degrees
     * @return The distance to the tag in meters
     */
    private double calculateDistanceToTag(double ty) {
        // Camera constants (adjust based on your robot's configuration)
        final double TARGET_HEIGHT_METERS = 0.5;  // Height of AprilTag from the floor
        final double CAMERA_HEIGHT_METERS = 0.5;  // Height of camera from the floor
        final double CAMERA_PITCH_RADIANS = 0;    // Angle of camera from horizontal (radians)
        
        return (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / 
               Math.tan(CAMERA_PITCH_RADIANS + Math.toRadians(ty));
    }
    
    /**
     * Resets the robot's position on the field.
     * 
     * @param pose The new pose
     */
    /**
     * Resets the robot's position on the field.
     * 
     * @param pose The new pose
     * @param gyro The current gyro angle (can be null to use current gyro reading)
     * @param modulePositions Current module positions (can be null to use current positions)
     */
    /**
     * Resets the robot's position on the field.
     * 
     * @param pose The new pose
     * @param gyro The current gyro angle (can be null to use current gyro reading)
     * @param modulePositions Current module positions (can be null to use current positions)
     */
    public void resetPosition(Pose2d pose, Rotation2d gyro, SwerveModulePosition[] modulePositions) {
        Rotation2d gyroToUse = (gyro != null) ? gyro : gyroAngle.get();
        SwerveModulePosition[] positionsToUse = (modulePositions != null) ? modulePositions : this.modulePositions.get();
        poseEstimator.resetPosition(gyroToUse, positionsToUse, pose);
    }
    
    /**
     * Resets the robot's position on the field using current gyro and module positions.
     * 
     * @param pose The new pose
     */
    public void resetPosition(Pose2d pose) {
        resetPosition(pose, null, null);
    }

    
    /**
     * Gets the robot's estimated position on the field.
     * 
     * @return The estimated pose
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    
    /**
     * Gets the robot's estimated position on the field as an Optional.
     * 
     * @return The estimated pose, or empty if not available
     */
    public Optional<Pose2d> getEstimatedPose() {
        return Optional.of(poseEstimator.getEstimatedPosition());
    }
    
    /**
     * Gets the robot's estimated position (x, y) on the field.
     * 
     * @return The estimated position
     */
    public Translation2d getEstimatedPosition() {
        return getPose().getTranslation();
    }
    
    /**
     * Gets the robot's estimated rotation.
     * 
     * @return The estimated rotation
     */
    public Rotation2d getEstimatedRotation() {
        return getPose().getRotation();
    }
    
    /**
     * Gets the current field-relative velocity of the robot.
     * 
     * @return The current velocity
     */
    public Translation2d getFieldRelativeVelocity() {
        // This is a simplified example - you'll need to implement this based on your robot's configuration
        // You might want to use the SwerveDriveKinematics to convert module states to chassis speeds
        return new Translation2d();
    }
}
