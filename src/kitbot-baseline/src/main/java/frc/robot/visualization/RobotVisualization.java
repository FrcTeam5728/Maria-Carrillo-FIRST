// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualization;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * 3D Robot Visualization for AdvantageScope
 * Provides robot position and orientation data for 3D visualization
 */
public class RobotVisualization {
  
  /**
   * Update robot position data for 3D visualization
   * This sends the robot pose to NetworkTables for AdvantageScope to visualize
   * 
   * @param robotPose Current robot pose
   */
  public static void updateRobotPosition(Pose2d robotPose) {
    // Send robot position to NetworkTables for AdvantageScope
    SmartDashboard.putNumber("Visualization/Robot/X", robotPose.getX());
    SmartDashboard.putNumber("Visualization/Robot/Y", robotPose.getY());
    SmartDashboard.putNumber("Visualization/Robot/Heading", robotPose.getRotation().getDegrees());
    
    // Send robot pose as a string for easy parsing
    SmartDashboard.putString("Visualization/Robot/Pose", 
        String.format("%.3f,%.3f,%.3f", 
            robotPose.getX(), 
            robotPose.getY(), 
            robotPose.getRotation().getDegrees()));
  }
  
  /**
   * Update robot dimensions for 3D visualization
   * 
   * @param length Robot length in meters
   * @param width Robot width in meters
   * @param height Robot height in meters
   */
  public static void updateRobotDimensions(double length, double width, double height) {
    SmartDashboard.putNumber("Visualization/Robot/Length", length);
    SmartDashboard.putNumber("Visualization/Robot/Width", width);
    SmartDashboard.putNumber("Visualization/Robot/Height", height);
  }
  
  /**
   * Update robot color for visualization
   * 
   * @param red Red component (0-1)
   * @param green Green component (0-1)
   * @param blue Blue component (0-1)
   */
  public static void updateRobotColor(double red, double green, double blue) {
    SmartDashboard.putNumber("Visualization/Robot/Color/Red", red);
    SmartDashboard.putNumber("Visualization/Robot/Color/Green", green);
    SmartDashboard.putNumber("Visualization/Robot/Color/Blue", blue);
  }
  
  /**
   * Initialize robot visualization with default KitBot dimensions
   */
  public static void initialize() {
    // KitBot dimensions (roughly 0.7m x 0.7m x 0.3m)
    updateRobotDimensions(0.7, 0.7, 0.3);
    
    // Blue robot color
    updateRobotColor(0.0, 0.0, 1.0);
    
    System.out.println("Robot 3D Visualization initialized for AdvantageScope");
  }
  
  /**
   * Send path data for visualization
   * 
   * @param pathPoints List of waypoints in the path
   */
  public static void updatePath(java.util.List<Pose2d> pathPoints) {
    double[] xCoords = new double[pathPoints.size()];
    double[] yCoords = new double[pathPoints.size()];
    
    for (int i = 0; i < pathPoints.size(); i++) {
      xCoords[i] = pathPoints.get(i).getX();
      yCoords[i] = pathPoints.get(i).getY();
    }
    
    SmartDashboard.putNumberArray("Visualization/Path/X", xCoords);
    SmartDashboard.putNumberArray("Visualization/Path/Y", yCoords);
    SmartDashboard.putNumber("Visualization/Path/Size", pathPoints.size());
  }
  
  /**
   * Clear path visualization
   */
  public static void clearPath() {
    SmartDashboard.putNumberArray("Visualization/Path/X", new double[0]);
    SmartDashboard.putNumberArray("Visualization/Path/Y", new double[0]);
    SmartDashboard.putNumber("Visualization/Path/Size", 0);
  }
  
  /**
   * Send target position for visualization
   * 
   * @param targetPosition Target position
   * @param targetType Type of target (e.g., "SCORING", "FUEL", "CLIMB")
   */
  public static void updateTarget(Translation2d targetPosition, String targetType) {
    SmartDashboard.putNumber("Visualization/Target/X", targetPosition.getX());
    SmartDashboard.putNumber("Visualization/Target/Y", targetPosition.getY());
    SmartDashboard.putString("Visualization/Target/Type", targetType);
  }
  
  /**
   * Clear target visualization
   */
  public static void clearTarget() {
    SmartDashboard.putNumber("Visualization/Target/X", 0.0);
    SmartDashboard.putNumber("Visualization/Target/Y", 0.0);
    SmartDashboard.putString("Visualization/Target/Type", "NONE");
  }
}
