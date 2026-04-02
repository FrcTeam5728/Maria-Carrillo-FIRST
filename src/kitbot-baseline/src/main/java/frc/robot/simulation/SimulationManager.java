// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.simulation.VirtualLimelight;

/**
 * Simulation Manager for Virtual Limelight System
 * Manages virtual hardware simulation and integration with robot code
 */
public class SimulationManager {
  
  private static SimulationManager instance;
  private VirtualLimelight virtualLimelight;
  private boolean isSimulation;
  private double lastUpdateTime;
  
  private SimulationManager() {
    this.isSimulation = RobotBase.isSimulation();
    this.lastUpdateTime = Timer.getFPGATimestamp();
    
    if (isSimulation) {
      initializeSimulation();
    }
  }
  
  /**
   * Get singleton instance
   */
  public static synchronized SimulationManager getInstance() {
    if (instance == null) {
      instance = new SimulationManager();
    }
    return instance;
  }
  
  /**
   * Initialize simulation components
   */
  private void initializeSimulation() {
    System.out.println("=== VIRTUAL LIMELIGHT SIMULATION ===");
    System.out.println("Initializing virtual Limelight system...");
    
    virtualLimelight = new VirtualLimelight();
    
    // Add simulation status to SmartDashboard
    SmartDashboard.putBoolean("Simulation/VirtualLimelight/Enabled", true);
    SmartDashboard.putString("Simulation/VirtualLimelight/Status", "Initialized");
    
    System.out.println("Virtual Limelight simulation ready");
    System.out.println("================================");
  }
  
  /**
   * Update simulation - should be called from robotPeriodic()
   */
  public void update() {
    if (!isSimulation || virtualLimelight == null) {
      return;
    }
    
    double currentTime = Timer.getFPGATimestamp();
    
    // Update at 50Hz (every 20ms)
    if (currentTime - lastUpdateTime >= 0.02) {
      updateSimulation();
      lastUpdateTime = currentTime;
    }
  }
  
  /**
   * Update simulation with current robot state
   */
  private void updateSimulation() {
    // Get robot pose from visualization system
    double robotX = SmartDashboard.getNumber("Visualization/Robot/X", 0.0);
    double robotY = SmartDashboard.getNumber("Visualization/Robot/Y", 0.0);
    double robotHeading = SmartDashboard.getNumber("Visualization/Robot/Heading", 0.0);
    
    Pose2d robotPose = new Pose2d(
        robotX, 
        robotY, 
        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(robotHeading)
    );
    
    // Update virtual Limelight
    virtualLimelight.update(robotPose);
    
    // Update simulation status
    SmartDashboard.putString("Simulation/VirtualLimelight/Status", "Running");
    SmartDashboard.putNumber("Simulation/VirtualLimelight/UpdateRate", 50.0);
  }
  
  /**
   * Check if running in simulation
   */
  public boolean isSimulation() {
    return isSimulation;
  }
  
  /**
   * Get virtual Limelight instance
   */
  public VirtualLimelight getVirtualLimelight() {
    return virtualLimelight;
  }
  
  /**
   * Enable/disable virtual Limelight
   */
  public void setVirtualLimelightEnabled(boolean enabled) {
    if (virtualLimelight != null) {
      virtualLimelight.setEnabled(enabled);
      SmartDashboard.putBoolean("Simulation/VirtualLimelight/Enabled", enabled);
    }
  }
  
  /**
   * Add custom target to simulation
   */
  public void addSimulationTarget(double x, double y, double width, double height, String type) {
    if (virtualLimelight != null) {
      edu.wpi.first.math.geometry.Translation2d position = 
          new edu.wpi.first.math.geometry.Translation2d(x, y);
      virtualLimelight.addTarget(position, width, height, type, 1.0);
      
      System.out.println("Added simulation target: " + type + " at (" + x + ", " + y + ")");
    }
  }
  
  /**
   * Remove simulation targets by type
   */
  public void removeSimulationTargets(String type) {
    if (virtualLimelight != null) {
      virtualLimelight.removeTargetsByType(type);
      System.out.println("Removed simulation targets: " + type);
    }
  }
  
  /**
   * Set virtual Limelight pipeline
   */
  public void setVirtualLimelightPipeline(int pipeline) {
    if (virtualLimelight != null) {
      virtualLimelight.setPipeline(pipeline);
      SmartDashboard.putNumber("Simulation/VirtualLimelight/Pipeline", pipeline);
    }
  }
}
