// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PulsingShooterSubsystem;

/**
 * Simple automatic shooting command that uses centralized Limelight data
 * and pulsing shooter for accurate fuel delivery.
 * 
 * Features:
 * - Uses single LimelightSubsystem for all vision data
 * - Pulsing shooter (3s ON, 0.5s OFF)
 * - Simple target acquisition and shooting
 * - Clear status feedback
 */
public class SimpleAutoShootCommand extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final PulsingShooterSubsystem shooterSubsystem;
    
    // Shooting state
    private boolean isAiming = false;
    private boolean isShooting = false;
    private double aimStartTime = 0;
    
    // Timing constants
    private static final double AIM_TIMEOUT = 2.0; // seconds
    private static final double ALIGNMENT_TOLERANCE = 3.0; // degrees
    
    /**
     * Creates a new SimpleAutoShootCommand.
     * 
     * @param driveSubsystem Drive subsystem for movement
     * @param limelightSubsystem Centralized Limelight data
     * @param shooterSubsystem Pulsing shooter subsystem
     */
    public SimpleAutoShootCommand(DriveSubsystem driveSubsystem, 
                                LimelightSubsystem limelightSubsystem,
                                PulsingShooterSubsystem shooterSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(driveSubsystem, limelightSubsystem, shooterSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("=== SIMPLE AUTO SHOOT COMMAND ===");
        System.out.println("Using centralized Limelight data");
        System.out.println("Pulsing shooter: 3s ON, 0.5s OFF");
        System.out.println("================================");
        
        isAiming = false;
        isShooting = false;
        aimStartTime = 0;
    }
    
    @Override
    public void execute() {
        // Get current target info from centralized Limelight
        LimelightSubsystem.TargetInfo targetInfo = limelightSubsystem.getTargetInfo();
        
        if (!targetInfo.hasTarget) {
            // No target - stop shooting and wait
            if (isShooting) {
                System.out.println("Target lost - stopping shooter");
                shooterSubsystem.stop();
                isShooting = false;
            }
            return;
        }
        
        // Target detected - proceed with shooting logic
        if (!isAiming && !isShooting) {
            // Start aiming at target
            startAiming();
        } else if (isAiming) {
            // Continue aiming or start shooting if aligned
            performAiming(targetInfo);
        } else if (isShooting) {
            // Continue shooting
            performShooting(targetInfo);
        }
    }
    
    /**
     * Starts the aiming process.
     */
    private void startAiming() {
        isAiming = true;
        aimStartTime = System.currentTimeMillis() / 1000.0;
        System.out.println("Started aiming at target: " + limelightSubsystem.getTargetInfo().toString());
    }
    
    /**
     * Performs aiming using drive system.
     * 
     * @param targetInfo Current target information
     */
    private void performAiming(LimelightSubsystem.TargetInfo targetInfo) {
        double aimDuration = (System.currentTimeMillis() / 1000.0) - aimStartTime;
        
        // Check for timeout
        if (aimDuration > AIM_TIMEOUT) {
            System.out.println("Aiming timeout - proceeding with shot");
            startShooting();
            return;
        }
        
        // Check if aligned with target
        if (Math.abs(targetInfo.horizontalOffset) < ALIGNMENT_TOLERANCE) {
            System.out.println("Target aligned - starting shooter");
            startShooting();
            return;
        }
        
        // Adjust robot to align with target
        double adjustment = calculateAlignmentAdjustment(targetInfo.horizontalOffset);
        // Use arcade drive for rotation (negative for clockwise)
        driveSubsystem.driveArcade(() -> 0.0, () -> -adjustment * 0.1);
        
        System.out.println("Aiming - Offset: " + String.format("%.1f", targetInfo.horizontalOffset) + 
                         "°, Adjustment: " + String.format("%.2f", adjustment) + "V");
    }
    
    /**
     * Calculates alignment adjustment for drive system.
     * 
     * @param horizontalOffset Horizontal offset to target in degrees
     * @return Voltage adjustment for tank drive
     */
    private double calculateAlignmentAdjustment(double horizontalOffset) {
        // Simple proportional control
        double kP = 0.1; // Proportional gain
        double adjustment = horizontalOffset * kP;
        
        // Limit to reasonable voltage range
        return Math.max(-3.0, Math.min(3.0, adjustment));
    }
    
    /**
     * Starts the shooting process.
     */
    private void startShooting() {
        isAiming = false;
        isShooting = true;
        
        // Start pulsing shooter
        shooterSubsystem.startPulsing();
        
        System.out.println("Started shooting with pulsing pattern");
        System.out.println("Target info: " + limelightSubsystem.getTargetInfo().toString());
    }
    
    /**
     * Continues shooting process.
     * 
     * @param targetInfo Current target information
     */
    private void performShooting(LimelightSubsystem.TargetInfo targetInfo) {
        // Check if target is still valid and aligned
        if (!targetInfo.hasTarget || Math.abs(targetInfo.horizontalOffset) > 10.0) {
            System.out.println("Target lost during shooting - stopping");
            shooterSubsystem.stop();
            isShooting = false;
            return;
        }
        
        // Pulsing shooter handles the 3s ON / 0.5s OFF pattern automatically
        // Just monitor and provide feedback
        
        if (shooterSubsystem.getPulseCount() % 5 == 0 && shooterSubsystem.getPulseCount() > 0) {
            // Log every 5 pulses
            System.out.println("Shooting progress - Pulses: " + shooterSubsystem.getPulseCount() + 
                             ", Status: " + shooterSubsystem.getStatus());
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Simple auto shoot command ended" + (interrupted ? " (interrupted)" : ""));
        
        // Always stop shooter when command ends
        shooterSubsystem.stop();
        // DriveSubsystem doesn't have stopMotor method, just stop the command
        
        // Reset state
        isAiming = false;
        isShooting = false;
        
        // Log final statistics
        System.out.println("Final stats - Pulses: " + shooterSubsystem.getPulseCount() + 
                         ", Total shots: " + shooterSubsystem.getTotalShotsFired());
    }
    
    @Override
    public boolean isFinished() {
        // Command never finishes automatically - must be manually cancelled
        return false;
    }
    
    /**
     * Gets current shooting status.
     * 
     * @return Status string
     */
    public String getStatus() {
        if (isAiming) {
            return "Aiming at target";
        } else if (isShooting) {
            return "Shooting - " + shooterSubsystem.getStatus();
        } else if (limelightSubsystem.hasTarget()) {
            return "Ready to shoot";
        } else {
            return "No target";
        }
    }
}
