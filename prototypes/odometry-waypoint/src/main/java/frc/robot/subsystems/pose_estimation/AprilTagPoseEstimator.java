package frc.robot.subsystems.pose_estimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Enhanced pose estimator that combines odometry with AprilTag vision measurements
 * for more accurate robot localization.
 */
public class AprilTagPoseEstimator extends OdometryPoseEstimator {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final LimelightSubsystem limelight;
    private boolean useVisionCorrection = true;
    
    // Standard deviations for pose estimation (x, y, theta)
    private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    private final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
    
    public AprilTagPoseEstimator(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelight) {
        super(swerveSubsystem);
        this.limelight = limelight;
        
        // Initialize the pose estimator with the swerve drive's kinematics
        this.poseEstimator = new SwerveDrivePoseEstimator(
            swerveSubsystem.getSwerveDrive().swerveDriveConfiguration.kinematics,
            swerveSubsystem.getHeading(),
            swerveSubsystem.getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs
        );
    }
    
    /**
     * Enable or disable vision-based pose correction.
     * @param enabled Whether to use vision correction
     */
    public void setVisionCorrectionEnabled(boolean enabled) {
        this.useVisionCorrection = enabled;
    }
    
    /**
     * Check if vision correction is enabled.
     * @return True if vision correction is enabled
     */
    public boolean isVisionCorrectionEnabled() {
        return useVisionCorrection;
    }
    
    @Override
    public void update() {
        super.update();
        
        // Update the pose estimator with odometry data
        poseEstimator.update(
            swerveSubsystem.getHeading(),
            swerveSubsystem.getModulePositions()
        );
        
        boolean hasVisionUpdate = false;
        // If vision correction is enabled and we have a target, add vision measurement
        if (useVisionCorrection && limelight.hasTarget()) {
            // Get the robot's pose relative to the AprilTag
            var visionPose = limelight.getRobotPose();
            if (visionPose.isPresent()) {
                double timestamp = Timer.getFPGATimestamp() - (limelight.getLatency() / 1000.0);
                // Add vision measurement with a timestamp
                poseEstimator.addVisionMeasurement(visionPose.get(), timestamp);
                hasVisionUpdate = true;
                
                // Log the vision update
                SmartDashboard.putNumber("Vision/LastUpdateTime", timestamp);
                SmartDashboard.putNumberArray("Vision/LastPose", 
                    new double[]{
                        visionPose.get().getX(),
                        visionPose.get().getY(),
                        visionPose.get().getRotation().getRadians()
                    });
            }
        }
        
        // Update SmartDashboard with vision correction status
        SmartDashboard.putBoolean("Vision/CorrectionEnabled", useVisionCorrection);
        SmartDashboard.putBoolean("Vision/HasTarget", limelight.hasTarget());
        SmartDashboard.putBoolean("Vision/Active", hasVisionUpdate);
        SmartDashboard.putNumber("Vision/LatencyMs", limelight.getLatency());
    }
    
    @Override
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }
    
    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
        poseEstimator.resetPosition(
            swerveSubsystem.getHeading(),
            swerveSubsystem.getModulePositions(),
            pose
        );
    }
}
