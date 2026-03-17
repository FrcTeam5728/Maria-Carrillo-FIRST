// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.FieldPositionSystem;

/**
 * Command to diagnose odometry system status.
 * Shows encoder, gyro, and vision data to identify why odometry might not be working.
 */
public class OdometryDiagnosticCommand extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final FieldPositionSystem fieldPositionSystem;
    private int printCounter = 0;
    
    /**
     * Creates a new OdometryDiagnosticCommand.
     * 
     * @param driveSubsystem Drive subsystem for odometry data
     * @param limelightSubsystem Vision subsystem for target data
     * @param fieldPositionSystem Field position system for comparison
     */
    public OdometryDiagnosticCommand(DriveSubsystem driveSubsystem, 
                                   LimelightSubsystem limelightSubsystem,
                                   FieldPositionSystem fieldPositionSystem) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.fieldPositionSystem = fieldPositionSystem;
        addRequirements(driveSubsystem, limelightSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("=== ODOMETRY DIAGNOSTIC START ===");
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
     * Prints comprehensive odometry diagnostic information.
     */
    private void printDiagnosticInfo() {
        System.out.println("--- Odometry Status ---");
        
        // DriveSubsystem odometry
        if (driveSubsystem != null) {
            var pose = driveSubsystem.getPose();
            System.out.println("DriveSubsystem Odometry:");
            System.out.println("  Position: (" + String.format("%.3f, %.3f", pose.getX(), pose.getY()) + ")");
            System.out.println("  Heading: " + String.format("%.1f°", pose.getRotation().getDegrees()));
            System.out.println("  Gyro Angle: " + String.format("%.1f°", driveSubsystem.getGyroAngle()));
        } else {
            System.out.println("DriveSubsystem: NULL");
        }
        
        // Limelight data
        if (limelightSubsystem != null) {
            System.out.println("Limelight Data:");
            System.out.println("  Connected: " + limelightSubsystem.isConnected());
            System.out.println("  Has Target: " + limelightSubsystem.hasTarget());
            if (limelightSubsystem.hasTarget()) {
                System.out.println("  Target ID: " + limelightSubsystem.getTargetId());
                System.out.println("  Distance: " + String.format("%.2f m", limelightSubsystem.getDistance()));
                System.out.println("  Horizontal Offset: " + String.format("%.1f°", limelightSubsystem.getHorizontalOffset()));
                System.out.println("  Vertical Offset: " + String.format("%.1f°", limelightSubsystem.getVerticalOffset()));
            }
        } else {
            System.out.println("LimelightSubsystem: NULL");
        }
        
        // FieldPositionSystem
        if (fieldPositionSystem != null) {
            var fieldPose = fieldPositionSystem.getRobotPose();
            System.out.println("FieldPositionSystem:");
            System.out.println("  Position: (" + String.format("%.3f, %.3f", fieldPose.getX(), fieldPose.getY()) + ")");
            System.out.println("  Confidence: " + String.format("%.2f", fieldPositionSystem.getConfidence()));
            System.out.println("  Source: " + fieldPositionSystem.getPositionSource());
        } else {
            System.out.println("FieldPositionSystem: NULL");
        }
        
        // Comparison
        System.out.println("Position Comparison:");
        if (driveSubsystem != null && fieldPositionSystem != null) {
            var drivePose = driveSubsystem.getPose();
            var fieldPose = fieldPositionSystem.getRobotPose();
            double distance = Math.sqrt(
                Math.pow(drivePose.getX() - fieldPose.getX(), 2) + 
                Math.pow(drivePose.getY() - fieldPose.getY(), 2)
            );
            System.out.println("  Position Difference: " + String.format("%.3f m", distance));
            System.out.println("  Heading Difference: " + 
                String.format("%.1f°", Math.abs(drivePose.getRotation().getDegrees() - fieldPose.getRotation().getDegrees())));
        }
        
        System.out.println("------------------------");
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("=== ODOMETRY DIAGNOSTIC END ===");
        System.out.println("Final status check:");
        printDiagnosticInfo();
        
        // Provide troubleshooting suggestions
        System.out.println("\n=== TROUBLESHOOTING SUGGESTIONS ===");
        
        if (driveSubsystem == null) {
            System.out.println("❌ DriveSubsystem is NULL - check initialization");
        } else {
            var pose = driveSubsystem.getPose();
            if (pose.getX() == 0.0 && pose.getY() == 0.0) {
                System.out.println("⚠️  Odometry at origin - robot may not be moving or encoders not working");
                System.out.println("   Check if robot is actually moving");
                System.out.println("   Check encoder connections");
                System.out.println("   Check gyro connection");
            } else {
                System.out.println("✅ DriveSubsystem odometry appears to be working");
            }
        }
        
        if (limelightSubsystem != null && !limelightSubsystem.hasTarget()) {
            System.out.println("⚠️  No Limelight target detected");
            System.out.println("   Point Limelight at AprilTag");
            System.out.println("   Check lighting conditions");
            System.out.println("   Verify AprilTag is visible");
        }
        
        if (fieldPositionSystem != null && fieldPositionSystem.getConfidence() < 0.5) {
            System.out.println("⚠️  Low field position confidence");
            System.out.println("   Move robot closer to target");
            System.out.println("   Check Limelight connection");
        }
    }
    
    @Override
    public boolean isFinished() {
        return false; // Run until manually cancelled
    }
}
