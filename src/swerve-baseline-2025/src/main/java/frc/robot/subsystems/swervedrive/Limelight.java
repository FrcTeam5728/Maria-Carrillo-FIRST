package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Limelight subsystem for vision processing and target tracking.
 * Provides access to Limelight camera data for targeting and pose estimation.
 */
public class Limelight extends SubsystemBase {
  
  private final NetworkTable limelightTable;
  private final String limelightName;
  
  // Constants for Limelight
  private static final double LIMELIGHT_MOUNT_HEIGHT = 0.5; // meters
  private static final double LIMELIGHT_ANGLE = 15.0; // degrees
  private static final double TARGET_HEIGHT = 0.0; // meters (adjust based on actual target)
  
  /**
   * Creates a new Limelight subsystem.
   * @param limelightName The name of the Limelight (default is "limelight")
   */
  public Limelight(String limelightName) {
    this.limelightName = limelightName;
    this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
  }
  
  /**
   * Creates a new Limelight subsystem with default name.
   */
  public Limelight() {
    this("limelight");
  }
  
  @Override
  public void periodic() {
    // Update dashboard with Limelight data
    SmartDashboard.putNumber("Limelight TX", getTx());
    SmartDashboard.putNumber("Limelight TY", getTy());
    SmartDashboard.putNumber("Limelight TA", getTa());
    SmartDashboard.putNumber("Limelight TV", getTv());
    SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
  }
  
  /**
   * Gets the horizontal offset from the crosshair to the target.
   * @return Horizontal offset in degrees (-29.8 to 29.8 degrees)
   */
  public double getTx() {
    return limelightTable.getEntry("tx").getDouble(0.0);
  }
  
  /**
   * Gets the vertical offset from the crosshair to the target.
   * @return Vertical offset in degrees (-24.85 to 24.85 degrees)
   */
  public double getTy() {
    return limelightTable.getEntry("ty").getDouble(0.0);
  }
  
  /**
   * Gets the target area (0% of image to 100% of image).
   * @return Target area as percentage (0.0 to 1.0)
   */
  public double getTa() {
    return limelightTable.getEntry("ta").getDouble(0.0);
  }
  
  /**
   * Gets whether the Limelight has any valid targets.
   * @return 1 if target is found, 0 if no target
   */
  public double getTv() {
    return limelightTable.getEntry("tv").getDouble(0.0);
  }
  
  /**
   * Checks if the Limelight has a valid target.
   * @return true if target is found, false otherwise
   */
  public boolean hasTarget() {
    return getTv() > 0.0;
  }
  
  /**
   * Gets the latency of the pipeline.
   * @return Pipeline latency in milliseconds
   */
  public double getLatency() {
    return limelightTable.getEntry("tl").getDouble(0.0);
  }
  
  /**
   * Gets the estimated distance to the target using simple trigonometry.
   * @return Estimated distance in meters
   */
  public double getDistanceToTarget() {
    if (!hasTarget()) return -1.0;
    
    double angleToTargetDegrees = LIMELIGHT_ANGLE + getTy();
    double angleToTargetRadians = Math.toRadians(angleToTargetDegrees);
    
    // Using tan to calculate distance: distance = (targetHeight - limelightHeight) / tan(angle)
    return (TARGET_HEIGHT - LIMELIGHT_MOUNT_HEIGHT) / Math.tan(angleToTargetRadians);
  }
  
  /**
   * Gets the pose from Limelight's MegaTag 2.0 system.
   * @return Optional containing the robot pose if available
   */
  public java.util.Optional<Pose2d> getBotPose() {
    double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    
    // Check if we have valid pose data (all zeros means no valid pose)
    if (botpose.length == 6 && (botpose[0] != 0 || botpose[1] != 0 || botpose[2] != 0)) {
      double x = botpose[0];
      double y = botpose[1];
      double rotation = botpose[5]; // yaw in degrees
      
      return java.util.Optional.of(new Pose2d(x, y, Rotation2d.fromDegrees(rotation)));
    }
    
    return java.util.Optional.empty();
  }
  
  /**
   * Gets the timestamp of the last pose measurement.
   * @return Timestamp in seconds
   */
  public double getBotPoseTimestamp() {
    return limelightTable.getEntry("botpose_wpiblue_timestamp").getDouble(0.0) / 1000.0;
  }
  
  /**
   * Sets the Limelight's pipeline.
   * @param pipeline Pipeline number (0-9)
   */
  public void setPipeline(int pipeline) {
    limelightTable.getEntry("pipeline").setNumber(pipeline);
  }
  
  /**
   * Sets the Limelight's LED mode.
   * @param mode LED mode (0=force off, 1=force blink, 2=force on, 3=use pipeline)
   */
  public void setLEDMode(int mode) {
    limelightTable.getEntry("ledMode").setNumber(mode);
  }
  
  /**
   * Sets the Limelight's streaming mode.
   * @param mode Streaming mode (0=standard, 1=PiP main, 2=PiP secondary)
   */
  public void setStreamingMode(int mode) {
    limelightTable.getEntry("stream").setNumber(mode);
  }
  
  /**
   * Sets the Limelight's snapshot mode.
   * @param mode Snapshot mode (0=stop taking snapshots, 1=take two snapshots)
   */
  public void setSnapshotMode(int mode) {
    limelightTable.getEntry("snapshot").setNumber(mode);
  }
  
  /**
   * Gets the Limelight's name.
   * @return The Limelight name
   */
  public String getLimelightName() {
    return limelightName;
  }
}
