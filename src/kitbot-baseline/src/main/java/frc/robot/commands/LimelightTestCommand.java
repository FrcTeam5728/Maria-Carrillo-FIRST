// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Simple command to test Limelight connectivity.
 * Tests NetworkTables connection and basic functionality.
 */
public class LimelightTestCommand extends Command {
    
    private final LimelightSubsystem limelightSubsystem;
    private boolean completed = false;
    
    /**
     * Creates a new LimelightTestCommand.
     * 
     * @param limelightSubsystem The Limelight subsystem to test
     */
    public LimelightTestCommand(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(limelightSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("=== LIMELIGHT CONNECTION TEST ===");
        completed = false;
    }
    
    @Override
    public void execute() {
        // Test basic connectivity
        boolean connected = limelightSubsystem.isConnected();
        boolean hasTarget = limelightSubsystem.hasTarget();
        
        System.out.println("Limelight Connection Status:");
        System.out.println("- Connected: " + connected);
        System.out.println("- Has Target: " + hasTarget);
        System.out.println("- Target ID: " + limelightSubsystem.getTargetId());
        System.out.println("- Horizontal Offset: " + limelightSubsystem.getHorizontalOffset() + "°");
        System.out.println("- Vertical Offset: " + limelightSubsystem.getVerticalOffset() + "°");
        System.out.println("- Target Area: " + limelightSubsystem.getTargetArea());
        System.out.println("- Distance: " + String.format("%.2f", limelightSubsystem.getDistance()) + "m");
        System.out.println("- Latency: " + String.format("%.1f", limelightSubsystem.getLatency()) + "ms");
        
        if (connected) {
            System.out.println("✅ Limelight is connected and responding!");
            
            if (hasTarget) {
                System.out.println("✅ Target detected!");
            } else {
                System.out.println("⚠️  No target detected - point Limelight at AprilTag");
            }
        } else {
            System.out.println("❌ Limelight not connected - check physical connection");
            System.out.println("Troubleshooting:");
            System.out.println("1. Check USB/Ethernet cable to Limelight");
            System.out.println("2. Verify Limelight power (LED should be on)");
            System.out.println("3. Check NetworkTables connection");
            System.out.println("4. Verify Limelight IP: 10.TE.AM.XX");
        }
        
        completed = true;
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("=== LIMELIGHT TEST COMPLETE ===");
    }
    
    @Override
    public boolean isFinished() {
        return completed;
    }
}
