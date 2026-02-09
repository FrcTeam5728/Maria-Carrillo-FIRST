package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team.kipling.limelight.Limelight;
import team.kipling.limelight.LimelightHelpers;
import team.kipling.limelight.LimelightHelpers.LimelightTarget_Fiducial;
import team.kipling.limelight.LimelightHelpers.Results;
import team.kipling.limelight.LimelightHelpers.Rotation3dHelpers;
import team.kipling.limelight.LimelightHelpers.Translation3dHelpers;
import team.kipling.limelight.LimelightPoseEstimate;
import team.kipling.limelight.LimelightPoseEstimate.PoseEstimate;

/**
 * Vision subsystem using Limelight for AprilTag detection and pose estimation.
 */
public class LimelightVision {

  /** April Tag Field Layout of the year. */
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  /** Maximum ambiguity threshold for pose estimation. */
  private final double maximumAmbiguity = 0.25;

  /** Current pose supplier, typically from SwerveDrive. */
  private final Supplier<Pose2d> currentPose;

  /** Field2d instance for visualization. */
  private final Field2d field2d;

  /** List of all Limelight cameras. */
  private final LimelightCamera[] cameras;

  /**
   * Constructor for the LimelightVision class.
   *
   * @param currentPose Supplier for the current robot pose
   * @param field Field2d instance for visualization
   * @param cameras Array of Limelight cameras to use
   */
  public LimelightVision(Supplier<Pose2d> currentPose, Field2d field, LimelightCamera... cameras) {
    this.currentPose = currentPose;
    this.field2d = field;
    this.cameras = cameras;
  }

  /**
   * Update the pose estimation using all configured cameras.
   *
   * @param swerveDrive SwerveDrive instance to update with vision measurements
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (Robot.isSimulation()) {
      // Update simulation if needed
      if (swerveDrive.getSimulationDriveTrainPose().isPresent()) {
        // In simulation, update the robot pose
        for (LimelightCamera camera : cameras) {
          camera.updateSimPose(swerveDrive.getSimulationDriveTrainPose().get());
        }
      }
    }

    // Process each camera
    for (LimelightCamera camera : cameras) {
      Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
      poseEst.ifPresent(
          pose -> {
            swerveDrive.addVisionMeasurement(
                pose.estimatedPose.toPose2d(), pose.timestampSeconds, camera.getCurrentStdDevs());
          });
    }
  }

  /**
   * Get the distance to a specific AprilTag.
   *
   * @param id AprilTag ID
   * @return Distance in meters, or -1 if tag not found
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag
        .map(pose3d -> currentPose.get().getTranslation().getDistance(pose3d.toPose2d().getTranslation()))
        .orElse(-1.0);
  }

  /**
   * Get the 2D pose of an AprilTag with an optional robot offset.
   *
   * @param aprilTag AprilTag ID
   * @param robotOffset Optional transform from the tag to the desired robot position
   * @return The target pose of the AprilTag
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout);
    }
  }

  /**
   * Represents a Limelight camera and its associated pose estimation.
   */
  public static class LimelightCamera {
    private final String cameraName;
    private final Transform3d robotToCamera;
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevs;
    private Matrix<N3, N1> currentStdDevs;
    private final Limelight limelight;
    private double lastUpdateTime = 0;

    /**
     * Create a new LimelightCamera instance.
     *
     * @param cameraName Name of the Limelight as configured in the web interface
     * @param robotToCamera Transform from the robot's center to the camera
     * @param singleTagStdDevs Standard deviations for single-tag pose estimation (x, y, theta)
     * @param multiTagStdDevs Standard deviations for multi-tag pose estimation (x, y, theta)
     */
    public LimelightCamera(
        String cameraName,
        Transform3d robotToCamera,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevs) {
      this.cameraName = cameraName;
      this.robotToCamera = robotToCamera;
      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevs;
      this.currentStdDevs = singleTagStdDevs;
      this.limelight = new Limelight(cameraName);
      
      // Configure pipeline for AprilTag detection (pipeline 0 is typically AprilTag)
      limelight.setPipeline(0);
      
      // Enable 3D visualization in the dashboard
      limelight.setCameraMode(Limelight.CAMMODE_VISION);
      limelight.setLEDMode(Limelight.LED_ON);
      limelight.setStreamMode(Limelight.STANDARD);
    }

    /**
     * Get the current standard deviations for pose estimation.
     *
     * @return Current standard deviations (x, y, theta)
     */
    public Matrix<N3, N1> getCurrentStdDevs() {
      return currentStdDevs;
    }

    /**
     * Update the simulated robot pose for this camera.
     *
     * @param robotPose Current robot pose in simulation
     */
    public void updateSimPose(Pose2d robotPose) {
      if (Robot.isSimulation()) {
        // In simulation, we can update the limelight's "simulated" view
        // This is a simplified version - in a real implementation, you'd want to simulate
        // the actual camera view based on the field layout and robot pose
        Pose3d robotPose3d = new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians())
        );
        
        // Transform to camera pose
        Pose3d cameraPose = robotPose3d.transformBy(robotToCamera);
        
        // Update the simulated camera pose
        // Note: This is a placeholder - in a real implementation, you'd need to simulate
        // the actual AprilTag detections based on the camera's field of view
        Logger.recordOutput("Limelight/" + cameraName + "/SimPose", cameraPose);
      }
    }

    /**
     * Get the estimated global pose from this camera.
     *
     * @return Optional containing the estimated pose and timestamp, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      // Get the latest result from the camera
      Results result = LimelightHelpers.getLatestResults(cameraName).targetingResults;
      
      // Check if we have any targets
      if (result.valid) {
        double timestamp = (result.timestamp_latency_ms / 1000.0) - (result.latency_capture / 1000.0);
        
        // Get the robot pose in field space
        LimelightPoseEstimate poseEstimate = limelight.getRobotPose();
        if (poseEstimate != null && poseEstimate.getPoseEstimate() != null) {
          PoseEstimate estimate = poseEstimate.getPoseEstimate();
          
          // Update standard deviations based on number of tags
          updateEstimationStdDevs(estimate.tagCount);
          
          // Create and return the estimated pose
          return Optional.of(
              new EstimatedRobotPose(
                  estimate.pose,
                  timestamp,
                  List.of() // Limelight doesn't provide direct access to targets used
              )
          );
        }
      }
      
      return Optional.empty();
    }

    /**
     * Update the standard deviations based on the number of tags detected.
     *
     * @param numTags Number of AprilTags detected
     */
    private void updateEstimationStdDevs(int numTags) {
      if (numTags == 0) {
        currentStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      } else if (numTags == 1) {
        currentStdDevs = singleTagStdDevs;
      } else {
        // For multiple tags, use the multi-tag standard deviations
        currentStdDevs = multiTagStdDevs;
      }
      
      // Log the current standard deviations
      Logger.recordOutput("Limelight/" + cameraName + "/StdDevs", 
          new double[]{
              currentStdDevs.get(0, 0), 
              currentStdDevs.get(1, 0), 
              currentStdDevs.get(2, 0)
          });
    }
  }

  /**
   * Represents an estimated robot pose with timestamp and targets used.
   */
  public static class EstimatedRobotPose {
    public final Pose3d estimatedPose;
    public final double timestampSeconds;
    public final List<Object> targetsUsed; // Using Object since Limelight doesn't provide direct access to targets

    public EstimatedRobotPose(Pose3d estimatedPose, double timestampSeconds, List<Object> targetsUsed) {
      this.estimatedPose = estimatedPose;
      this.timestampSeconds = timestampSeconds;
      this.targetsUsed = targetsUsed;
    }
  }
}
