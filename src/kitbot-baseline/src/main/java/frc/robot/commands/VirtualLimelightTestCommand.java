// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.simulation.SimulationManager;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to test virtual Limelight functionality
 * Demonstrates virtual Limelight capabilities and provides debugging information
 */
public class VirtualLimelightTestCommand extends Command {
  
  private final LimelightSubsystem limelightSubsystem;
  private final SimulationManager simulationManager;
  private int testPhase;
  private double testStartTime;
  
  public VirtualLimelightTestCommand(LimelightSubsystem limelightSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    this.simulationManager = SimulationManager.getInstance();
    addRequirements(limelightSubsystem);
  }
  
  @Override
  public void initialize() {
    testPhase = 0;
    testStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    
    DriverStation.reportWarning("=== VIRTUAL LIMELIGHT TEST STARTED ===", false);
    System.out.println("Virtual Limelight Test Command initialized");
    
    // Add test targets
    addTestTargets();
  }
  
  @Override
  public void execute() {
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double elapsedTime = currentTime - testStartTime;
    
    switch (testPhase) {
      case 0:
        // Phase 1: Basic target detection test (0-5 seconds)
        if (elapsedTime < 5.0) {
          runBasicDetectionTest();
        } else {
          testPhase = 1;
          System.out.println("Phase 1 complete, starting Phase 2...");
        }
        break;
        
      case 1:
        // Phase 2: Pipeline switching test (5-10 seconds)
        if (elapsedTime < 10.0) {
          runPipelineTest(elapsedTime);
        } else {
          testPhase = 2;
          System.out.println("Phase 2 complete, starting Phase 3...");
        }
        break;
        
      case 2:
        // Phase 3: Dynamic target test (10-15 seconds)
        if (elapsedTime < 15.0) {
          runDynamicTargetTest(elapsedTime);
        } else {
          testPhase = 3;
          System.out.println("Phase 3 complete, finishing test...");
        }
        break;
        
      case 3:
        // Test complete
        if (elapsedTime >= 15.0) {
          cancel();
        }
        break;
    }
    
    // Update test status
    updateTestStatus(elapsedTime);
  }
  
  private void runBasicDetectionTest() {
    // Check if Limelight is detecting targets
    boolean hasTarget = limelightSubsystem.hasTarget();
    double tx = limelightSubsystem.getHorizontalOffset();
    double ty = limelightSubsystem.getVerticalOffset();
    double ta = limelightSubsystem.getTargetArea();
    
    SmartDashboard.putBoolean("Test/HasTarget", hasTarget);
    SmartDashboard.putNumber("Test/TX", tx);
    SmartDashboard.putNumber("Test/TY", ty);
    SmartDashboard.putNumber("Test/TA", ta);
    
    if (hasTarget) {
      System.out.printf("Target detected: TX=%.2f, TY=%.2f, TA=%.2f%n", tx, ty, ta);
    }
  }
  
  private void runPipelineTest(double elapsedTime) {
    // Switch between pipelines every second
    int pipeline = ((int) (elapsedTime - 5.0)) % 3;
    simulationManager.setVirtualLimelightPipeline(pipeline);
    
    SmartDashboard.putNumber("Test/Pipeline", pipeline);
    System.out.println("Testing pipeline: " + pipeline);
  }
  
  private void runDynamicTargetTest(double elapsedTime) {
    // Add moving targets
    double timeInPhase = elapsedTime - 10.0;
    double targetX = 2.0 + Math.sin(timeInPhase * 0.5) * 1.5;
    double targetY = 1.0 + Math.cos(timeInPhase * 0.3) * 0.8;
    
    // Remove old dynamic targets and add new ones
    simulationManager.removeSimulationTargets("DYNAMIC");
    simulationManager.addSimulationTarget(targetX, targetY, 0.5, 0.5, "DYNAMIC");
    
    SmartDashboard.putNumber("Test/DynamicTargetX", targetX);
    SmartDashboard.putNumber("Test/DynamicTargetY", targetY);
  }
  
  private void updateTestStatus(double elapsedTime) {
    SmartDashboard.putNumber("Test/ElapsedTime", elapsedTime);
    SmartDashboard.putNumber("Test/Phase", testPhase);
    SmartDashboard.putString("Test/Status", getPhaseDescription(testPhase));
  }
  
  private String getPhaseDescription(int phase) {
    switch (phase) {
      case 0: return "Basic Detection";
      case 1: return "Pipeline Switching";
      case 2: return "Dynamic Targets";
      case 3: return "Complete";
      default: return "Unknown";
    }
  }
  
  private void addTestTargets() {
    // Add test targets at known positions
    simulationManager.addSimulationTarget(3.0, 1.0, 0.8, 0.6, "TEST_SPEAKER");
    simulationManager.addSimulationTarget(3.0, -1.0, 0.6, 0.4, "TEST_AMP");
    simulationManager.addSimulationTarget(1.5, 0.5, 0.3, 0.2, "TEST_FUEL");
    
    System.out.println("Test targets added to simulation");
  }
  
  @Override
  public void end(boolean interrupted) {
    // Clean up test targets
    simulationManager.removeSimulationTargets("TEST_SPEAKER");
    simulationManager.removeSimulationTargets("TEST_AMP");
    simulationManager.removeSimulationTargets("TEST_FUEL");
    simulationManager.removeSimulationTargets("DYNAMIC");
    
    // Reset pipeline
    simulationManager.setVirtualLimelightPipeline(0);
    
    DriverStation.reportWarning("=== VIRTUAL LIMELIGHT TEST COMPLETED ===", false);
    System.out.println("Virtual Limelight Test Command ended");
  }
  
  @Override
  public boolean isFinished() {
    return testPhase >= 3 && 
           edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - testStartTime >= 15.0;
  }
}
