// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.StringPublisher;

import java.util.Random;
import java.util.List;
import java.util.ArrayList;

/**
 * Virtual Limelight Simulation System
 * Simulates Limelight vision system for testing without physical hardware
 * Provides realistic target detection, tracking, and network table data
 */
public class VirtualLimelight {
  
  // NetworkTable publishers
  private NetworkTable limelightTable;
  private DoublePublisher txPublisher;
  private DoublePublisher tyPublisher;
  private DoublePublisher taPublisher;
  private DoublePublisher tsPublisher;
  private DoublePublisher tlPublisher;
  private DoublePublisher tshortPublisher;
  private DoublePublisher tlongPublisher;
  private DoublePublisher thorPublisher;
  private DoublePublisher tvertPublisher;
  private DoublePublisher getpipePublisher;
  private IntegerPublisher pipelinePublisher;
  private BooleanPublisher validPublisher;
  private StringPublisher jsonPublisher;
  private DoubleArrayPublisher camtranPublisher;
  
  // Simulation parameters
  private Pose2d robotPose;
  private List<SimulatedTarget> targets;
  private Random random;
  private boolean isEnabled;
  private int currentPipeline;
  
  // Limelight configuration
  private static final double HORIZONTAL_FOV = 59.6; // degrees
  private static final double VERTICAL_FOV = 49.7;   // degrees
  private static final double MOUNT_HEIGHT = 0.5;   // meters
  private static final double MOUNT_ANGLE = 20.0;  // degrees up
  private static final double MAX_RANGE = 10.0;     // meters
  
  public VirtualLimelight() {
    initializeNetworkTables();
    initializeTargets();
    this.robotPose = new Pose2d();
    this.random = new Random();
    this.isEnabled = true;
    this.currentPipeline = 0;
  }
  
  /**
   * Initialize NetworkTable publishers for Limelight data
   */
  private void initializeNetworkTables() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    limelightTable = inst.getTable("limelight");
    
    txPublisher = limelightTable.getDoubleTopic("tx").publish();
    tyPublisher = limelightTable.getDoubleTopic("ty").publish();
    taPublisher = limelightTable.getDoubleTopic("ta").publish();
    tsPublisher = limelightTable.getDoubleTopic("ts").publish();
    tlPublisher = limelightTable.getDoubleTopic("tl").publish();
    tshortPublisher = limelightTable.getDoubleTopic("tshort").publish();
    tlongPublisher = limelightTable.getDoubleTopic("tlong").publish();
    thorPublisher = limelightTable.getDoubleTopic("thor").publish();
    tvertPublisher = limelightTable.getDoubleTopic("tvert").publish();
    getpipePublisher = limelightTable.getDoubleTopic("getpipe").publish();
    pipelinePublisher = limelightTable.getIntegerTopic("pipeline").publish();
    validPublisher = limelightTable.getBooleanTopic("tv").publish();
    jsonPublisher = limelightTable.getStringTopic("json").publish();
    camtranPublisher = limelightTable.getDoubleArrayTopic("camtran").publish();
  }
  
  /**
   * Initialize simulated targets on the field
   */
  private void initializeTargets() {
    targets = new ArrayList<>();
    
    // Add speaker targets (high goals)
    targets.add(new SimulatedTarget(
        new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162)),
        Units.inchesToMeters(24), // 2ft diameter
        Units.inchesToMeters(18), // 1.5ft tall
        "SPEAKER",
        1.0
    ));
    
    // Add amp targets (side goals)
    targets.add(new SimulatedTarget(
        new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(-162)),
        Units.inchesToMeters(30), // 2.5ft diameter
        Units.inchesToMeters(12), // 1ft tall
        "AMP",
        0.8
    ));
    
    // Add game pieces for fuel collection
    for (int i = 0; i < 5; i++) {
      double x = Units.inchesToMeters(100 + i * 40);
      double y = Units.inchesToMeters(-60 + (i % 3) * 60);
      targets.add(new SimulatedTarget(
          new Translation2d(x, y),
          Units.inchesToMeters(14), // Note diameter
          Units.inchesToMeters(2),  // Note height
          "FUEL",
          0.6
      ));
    }
  }
  
  /**
   * Update virtual Limelight with current robot position
   */
  public void update(Pose2d robotPose) {
    this.robotPose = robotPose;
    
    if (!isEnabled) {
      publishNoTarget();
      return;
    }
    
    // Find best target in view
    SimulatedTarget bestTarget = findBestTarget();
    
    if (bestTarget != null) {
      publishTargetData(bestTarget);
    } else {
      publishNoTarget();
    }
    
    // Update pipeline info
    getpipePublisher.set(currentPipeline);
    pipelinePublisher.set(currentPipeline);
  }
  
  /**
   * Find the best target to track based on robot position and orientation
   */
  private SimulatedTarget findBestTarget() {
    SimulatedTarget bestTarget = null;
    double bestScore = -1.0;
    
    for (SimulatedTarget target : targets) {
      if (!target.isActive()) continue;
      
      // Calculate distance and angle to target
      Translation2d toTarget = target.getPosition().minus(robotPose.getTranslation());
      double distance = toTarget.getNorm();
      double angle = robotPose.getRotation().getDegrees() - 
                   Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
      
      // Normalize angle to [-180, 180]
      angle = ((angle + 180) % 360) - 180;
      
      // Check if target is in range and field of view
      if (distance > MAX_RANGE || Math.abs(angle) > HORIZONTAL_FOV / 2) {
        continue;
      }
      
      // Calculate target score (closer and more centered is better)
      double score = target.getPriority() * (1.0 - distance / MAX_RANGE) * 
                    (1.0 - Math.abs(angle) / (HORIZONTAL_FOV / 2));
      
      // Add some noise for realism
      score += (random.nextGaussian() * 0.05);
      
      if (score > bestScore) {
        bestScore = score;
        bestTarget = target;
      }
    }
    
    return bestTarget;
  }
  
  /**
   * Publish target data to NetworkTables
   */
  private void publishTargetData(SimulatedTarget target) {
    Translation2d toTarget = target.getPosition().minus(robotPose.getTranslation());
    double distance = toTarget.getNorm();
    
    // Calculate angles
    double angleToTarget = Math.atan2(toTarget.getY(), toTarget.getX());
    double robotAngle = robotPose.getRotation().getRadians();
    double tx = Math.toDegrees(angleToTarget - robotAngle);
    
    // Calculate vertical angle based on height difference
    double heightDiff = target.getHeight() - MOUNT_HEIGHT;
    double ty = Math.toDegrees(Math.atan2(heightDiff, distance)) - MOUNT_ANGLE;
    
    // Calculate target area (0-100% of screen)
    double apparentWidth = 2 * Math.atan(target.getWidth() / (2 * distance));
    double apparentHeight = 2 * Math.atan(target.getHeight() / (2 * distance));
    double ta = (apparentWidth * apparentHeight) / (HORIZONTAL_FOV * VERTICAL_FOV) * 100;
    
    // Add realistic noise
    tx += random.nextGaussian() * 0.1;
    ty += random.nextGaussian() * 0.1;
    ta = Math.max(0, ta + random.nextGaussian() * 0.5);
    
    // Calculate other target measurements
    double ts = target.getSkew();
    double tl = target.getLatency();
    double tshort = Math.min(apparentWidth, apparentHeight);
    double tlong = Math.max(apparentWidth, apparentHeight);
    double thor = apparentWidth;
    double tvert = apparentHeight;
    
    // Camera transform (simplified)
    double[] camtran = {
        toTarget.getX(), toTarget.getY(), heightDiff,
        Math.toDegrees(robotAngle), tx, ty
    };
    
    // Create JSON output
    String json = String.format(
        "{\"class\":\"LTTarget\",\"tx\":%.2f,\"ty\":%.2f,\"ta\":%.2f,\"ts\":%.2f,\"tl\":%.2f,\"tshort\":%.2f,\"tlong\":%.2f,\"thor\":%.2f,\"tvert\":%.2f,\"class\":\"LTTarget\"}",
        tx, ty, ta, ts, tl, tshort, tlong, thor, tvert
    );
    
    // Publish all values
    txPublisher.set(tx);
    tyPublisher.set(ty);
    taPublisher.set(ta);
    tsPublisher.set(ts);
    tlPublisher.set(tl);
    tshortPublisher.set(tshort);
    tlongPublisher.set(tlong);
    thorPublisher.set(thor);
    tvertPublisher.set(tvert);
    validPublisher.set(true);
    jsonPublisher.set(json);
    camtranPublisher.set(camtran);
  }
  
  /**
   * Publish no-target data to NetworkTables
   */
  private void publishNoTarget() {
    txPublisher.set(0.0);
    tyPublisher.set(0.0);
    taPublisher.set(0.0);
    tsPublisher.set(0.0);
    tlPublisher.set(0.0);
    tshortPublisher.set(0.0);
    tlongPublisher.set(0.0);
    thorPublisher.set(0.0);
    tvertPublisher.set(0.0);
    validPublisher.set(false);
    jsonPublisher.set("[]");
    camtranPublisher.set(new double[0]);
  }
  
  /**
   * Enable/disable virtual Limelight
   */
  public void setEnabled(boolean enabled) {
    this.isEnabled = enabled;
  }
  
  /**
   * Set current pipeline
   */
  public void setPipeline(int pipeline) {
    this.currentPipeline = pipeline;
  }
  
  /**
   * Add a custom target to the simulation
   */
  public void addTarget(Translation2d position, double width, double height, String type, double priority) {
    targets.add(new SimulatedTarget(position, width, height, type, priority));
  }
  
  /**
   * Remove all targets of a specific type
   */
  public void removeTargetsByType(String type) {
    targets.removeIf(target -> target.getType().equals(type));
  }
  
  /**
   * Get current robot pose
   */
  public Pose2d getRobotPose() {
    return robotPose;
  }
  
  /**
   * Simulated target class
   */
  private static class SimulatedTarget {
    private final Translation2d position;
    private final double width;
    private final double height;
    private final String type;
    private final double priority;
    private boolean active;
    
    public SimulatedTarget(Translation2d position, double width, double height, String type, double priority) {
      this.position = position;
      this.width = width;
      this.height = height;
      this.type = type;
      this.priority = priority;
      this.active = true;
    }
    
    public Translation2d getPosition() { return position; }
    public double getWidth() { return width; }
    public double getHeight() { return height; }
    public String getType() { return type; }
    public double getPriority() { return priority; }
    public boolean isActive() { return active; }
    public double getSkew() { return 0.0; } // Simplified skew
    public double getLatency() { return 11.0; } // 11ms latency
    
    public void setActive(boolean active) { this.active = active; }
  }
}
