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
        // Test raw NetworkTables connection first
        boolean rawConnection = limelightSubsystem.testRawConnection();
        
        // Test basic connectivity
        boolean connected = limelightSubsystem.isConnected();
        boolean hasTarget = limelightSubsystem.hasTarget();
        
        System.out.println("\n=== LIMELIGHT CONNECTION TEST RESULTS ===");
        System.out.println("Raw NetworkTables: " + (rawConnection ? "WORKING" : "FAILED"));
        System.out.println("Connection Status: " + (connected ? "CONNECTED" : "DISCONNECTED"));
        System.out.println("Target Detection: " + (hasTarget ? "TARGET FOUND" : "NO TARGET"));
        
        if (hasTarget) {
            System.out.println("\n--- Target Details ---");
            System.out.println("Target ID: " + limelightSubsystem.getTargetId());
            System.out.println("Horizontal Offset: " + String.format("%.1f", limelightSubsystem.getHorizontalOffset()) + "°");
            System.out.println("Vertical Offset: " + String.format("%.1f", limelightSubsystem.getVerticalOffset()) + "°");
            System.out.println("Target Area: " + String.format("%.2f", limelightSubsystem.getTargetArea()));
            System.out.println("Distance: " + String.format("%.2f", limelightSubsystem.getDistance()) + "m");
            System.out.println("Latency: " + String.format("%.1f", limelightSubsystem.getLatency()) + "ms");
        }
        
        if (rawConnection && !connected) {
            System.out.println("\n⚠️ NetworkTables working but Limelight not connected");
            System.out.println("Check Limelight device:");
            System.out.println("  - Power: Is Limelight LED on?");
            System.out.println("  - Network: Is Ethernet cable connected?");
            System.out.println("  - IP: Is Limelight on 10.57.28.11?");
            System.out.println("  - Team: Configured for team 5728?");
        } else if (!rawConnection) {
            System.out.println("\n❌ NetworkTables connection failed");
            System.out.println("Check robot network:");
            System.out.println("  - Robot connected to network?");
            System.out.println("  - NetworkTables server running?");
            System.out.println("  - Firewall blocking NetworkTables?");
        } else if (connected && !hasTarget) {
            System.out.println("\n✅ Limelight connected - no target detected");
            System.out.println("Point Limelight at AprilTag to test target detection");
        } else {
            System.out.println("\n✅ Limelight fully operational!");
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
