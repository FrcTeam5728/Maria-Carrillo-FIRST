// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.AprilTagSubsystem;

/**
 * Command that runs a comprehensive Limelight diagnostic test.
 * Useful for troubleshooting USB connection and configuration issues.
 */
public class LimelightDiagnosticCommand extends Command {
    
    private final AprilTagSubsystem aprilTagSubsystem;
    private boolean hasRun = false;
    
    /**
     * Creates a new LimelightDiagnosticCommand.
     * 
     * @param aprilTagSubsystem The AprilTag subsystem to test
     */
    public LimelightDiagnosticCommand(AprilTagSubsystem aprilTagSubsystem) {
        this.aprilTagSubsystem = aprilTagSubsystem;
    }
    
    @Override
    public void initialize() {
        System.out.println("\n=== LIMELIGHT DIAGNOSTIC START ===");
        
        // Test basic connection
        boolean isConnected = aprilTagSubsystem.isLimelightConnected();
        System.out.println("Connection Status: " + (isConnected ? "CONNECTED" : "NOT CONNECTED"));
        
        // Test target detection
        boolean hasTarget = aprilTagSubsystem.hasTarget();
        System.out.println("Target Detection: " + (hasTarget ? "TARGET FOUND" : "NO TARGET"));
        
        if (hasTarget) {
            System.out.println("Target ID: " + aprilTagSubsystem.getTargetId());
            System.out.println("Horizontal Offset: " + aprilTagSubsystem.getTargetX() + " degrees");
            System.out.println("Vertical Offset: " + aprilTagSubsystem.getTargetY() + " degrees");
            System.out.println("Target Area: " + aprilTagSubsystem.getTargetArea() + "%");
            System.out.println("Distance to Target: " + aprilTagSubsystem.getDistanceToTarget() + " meters");
        }
        
        // Troubleshooting checklist
        System.out.println("\n--- TROUBLESHOOTING CHECKLIST ---");
        if (!isConnected) {
            System.out.println("❌ LIMELIGHT NOT RESPONDING");
            System.out.println("Check the following:");
            System.out.println("1. Power: Limelight LED should be lit (usually green/blue)");
            System.out.println("2. USB Connection: Cable firmly connected to both Limelight and roboRIO");
            System.out.println("3. USB Port: Try different USB port on roboRIO");
            System.out.println("4. NetworkTables: Verify NetworkTables are working");
            System.out.println("5. Configuration: Limelight configured with name 'limelight'");
            System.out.println("6. Firmware: Limelight firmware is up to date");
        } else if (!hasTarget) {
            System.out.println("⚠️  LIMELIGHT CONNECTED BUT NO TARGET");
            System.out.println("Check the following:");
            System.out.println("1. AprilTags: Are there AprilTags in view?");
            System.out.println("2. Lighting: Good lighting conditions?");
            System.out.println("3. Pipeline: Correct pipeline selected (0-9)?");
            System.out.println("4. Mode: Not in driver mode (should be vision mode)");
            System.out.println("5. Distance: Not too far from AprilTags?");
        } else {
            System.out.println("✅ LIMELIGHT WORKING CORRECTLY");
        }
        
        System.out.println("--- USB CONNECTION TIPS ---");
        System.out.println("• Use high-quality USB cable (not too long)");
        System.out.println("• Avoid USB hubs - connect directly to roboRIO");
        System.out.println("• Check for bent pins in USB connectors");
        System.out.println("• Try power cycling robot and Limelight");
        System.out.println("• Verify roboRIO USB port is enabled in code");
        
        System.out.println("=== LIMELIGHT DIAGNOSTIC END ===\n");
        
        hasRun = true;
    }
    
    @Override
    public boolean isFinished() {
        return hasRun;
    }
}
