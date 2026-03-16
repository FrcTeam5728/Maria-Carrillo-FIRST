// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to diagnose Limelight connection and status.
 * Provides detailed information about Limelight connectivity and target detection.
 */
public class LimelightDiagnosticCommand extends Command {
    
    private final LimelightSubsystem limelightSubsystem;
    private int printCounter = 0;
    
    /**
     * Creates a new LimelightDiagnosticCommand.
     * 
     * @param limelightSubsystem The Limelight subsystem to diagnose
     */
    public LimelightDiagnosticCommand(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(limelightSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("=== LIMELIGHT DIAGNOSTIC START ===");
        System.out.println("Checking Limelight connection and status...");
        printCounter = 0;
    }
    
    @Override
    public void execute() {
        // Print status every second (assuming 50Hz periodic)
        if (printCounter % 50 == 0) {
            printDiagnosticInfo();
        }
        printCounter++;
    }
    
    /**
     * Prints comprehensive diagnostic information.
     */
    private void printDiagnosticInfo() {
        System.out.println("--- Limelight Status ---");
        System.out.println("Connected: " + limelightSubsystem.isConnected());
        System.out.println("Has Target: " + limelightSubsystem.hasTarget());
        
        if (limelightSubsystem.hasTarget()) {
            System.out.println("Target ID: " + limelightSubsystem.getTargetId());
            System.out.println("Horizontal Offset: " + String.format("%.2f", limelightSubsystem.getHorizontalOffset()) + "°");
            System.out.println("Vertical Offset: " + String.format("%.2f", limelightSubsystem.getVerticalOffset()) + "°");
            System.out.println("Target Area: " + String.format("%.2f", limelightSubsystem.getTargetArea()));
            System.out.println("Distance: " + String.format("%.2f", limelightSubsystem.getDistance()) + "m");
        } else {
            System.out.println("No target detected");
        }
        
        System.out.println("Latency: " + String.format("%.1f", limelightSubsystem.getLatency()) + "ms");
        System.out.println("------------------------");
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("=== LIMELIGHT DIAGNOSTIC END ===");
        System.out.println("Final status check:");
        printDiagnosticInfo();
        
        // Provide troubleshooting suggestions
        if (!limelightSubsystem.isConnected()) {
            System.out.println("\n=== TROUBLESHOOTING SUGGESTIONS ===");
            System.out.println("1. Check USB/Ethernet connection to Limelight");
            System.out.println("2. Verify Limelight power (LED should be on)");
            System.out.println("3. Check NetworkTables connection");
            System.out.println("4. Verify Limelight IP address (usually 10.TE.AM.XX)");
            System.out.println("5. Check firewall settings");
        } else if (!limelightSubsystem.hasTarget()) {
            System.out.println("\n=== TARGET DETECTION ISSUES ===");
            System.out.println("1. Point Limelight at AprilTag");
            System.out.println("2. Check if AprilTag is visible");
            System.out.println("3. Verify correct pipeline is selected");
            System.out.println("4. Check lighting conditions");
            System.out.println("5. Verify AprilTag is not too far away");
        }
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until manually cancelled
    }
}
