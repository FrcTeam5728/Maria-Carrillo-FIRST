package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTable table;
  private final String tableName;
  
  // NetworkTable entries
  private final NetworkTableEntry tv;  // Whether the limelight has any valid targets (0 or 1)
  private final NetworkTableEntry tx;  // Horizontal offset from crosshair to target (-27 to 27 degrees)
  private final NetworkTableEntry ty;  // Vertical offset from crosshair to target (-20.5 to 20.5 degrees)
  private final NetworkTableEntry ta;  // Target area (0% to 100% of image)
  private final NetworkTableEntry tid; // Target ID (which AprilTag)
  private final NetworkTableEntry botpose; // Robot's position in field space
  private final NetworkTableEntry targetpose; // Target's position in camera space

  // Camera parameters (you may need to adjust these based on your setup)
  private static final double CAMERA_HEIGHT_METERS = 0.5;  // Height of camera from ground
  private static final double TARGET_HEIGHT_METERS = 1.0;  // Height of target from ground
  private static final double CAMERA_PITCH_RADIANS = Math.toRadians(0);  // Angle of camera
  
  public LimelightSubsystem() {
    this("limelight");
  }
  
  public LimelightSubsystem(String tableName) {
    this.tableName = tableName;
    this.table = NetworkTableInstance.getDefault().getTable(tableName);
    
    // Initialize NetworkTable entries
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tid = table.getEntry("tid");
    botpose = table.getEntry("botpose");
    targetpose = table.getEntry("targetpose");
    
    // Set pipeline to AprilTag detection (pipeline 0)
    setPipeline(0);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }
  
  private void updateSmartDashboard() {
    SmartDashboard.putBoolean(tableName + "/Has Target", hasTarget());
    SmartDashboard.putNumber(tableName + "/X", getTx());
    SmartDashboard.putNumber(tableName + "/Y", getTy());
    SmartDashboard.putNumber(tableName + "/Area", getTa());
    SmartDashboard.putNumber(tableName + "/Target ID", getTargetId());
    SmartDashboard.putNumber(tableName + "/Distance", getDistanceToTarget());
  }

  // Basic target detection
  public boolean hasTarget() {
    return tv.getDouble(0) == 1.0;
  }
  
  public double getTx() {
    return tx.getDouble(0.0);
  }
  
  public double getTy() {
    return ty.getDouble(0.0);
  }
  
  public double getTa() {
    return ta.getDouble(0.0);
  }
  
  public int getTargetId() {
    return (int)tid.getInteger(-1);
  }
  
  // Pipeline control
  public void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }
  
  // LED control
  public void setLedMode(LedMode mode) {
    table.getEntry("ledMode").setNumber(mode.value);
  }
  
  // Camera mode control
  public void setCamMode(CamMode mode) {
    table.getEntry("camMode").setNumber(mode.value);
  }
  
  // Get 3D pose of the robot (if using AprilTags)
  public Pose3d getRobotPose() {
    double[] poseArray = botpose.getDoubleArray(new double[6]);
    if (poseArray.length < 6) return new Pose3d();
    
    return new Pose3d(
      new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
      new Rotation3d(poseArray[3], poseArray[4], poseArray[5])
    );
  }
  
  // Get 3D pose of the target (if using AprilTags)
  public Pose3d getTargetPose() {
    double[] poseArray = targetpose.getDoubleArray(new double[6]);
    if (poseArray.length < 6) return new Pose3d();
    
    return new Pose3d(
      new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
      new Rotation3d(poseArray[3], poseArray[4], poseArray[5])
    );
  }
  
  // Calculate distance to target using trig (works for reflective tape too)
  public double getDistanceToTarget() {
    if (!hasTarget()) return -1.0;
    
    double angleToTarget = Math.toRadians(getTy() + CAMERA_PITCH_RADIANS);
    double heightDifference = TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS;
    
    // Make sure we don't divide by zero or get invalid results
    if (Math.abs(angleToTarget) < 0.1) return Double.MAX_VALUE;
    
    return heightDifference / Math.tan(angleToTarget);
  }
  
  // Enums for Limelight settings
  public enum LedMode {
    PIPELINE(0),
    OFF(1),
    BLINK(2),
    ON(3);
    
    public final int value;
    
    LedMode(int value) {
      this.value = value;
    }
  }
  
  public enum CamMode {
    VISION(0),
    DRIVER(1);
    
    public final int value;
    
    CamMode(int value) {
      this.value = value;
    }
  }
}