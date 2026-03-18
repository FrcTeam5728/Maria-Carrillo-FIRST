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
    private double shootingStartTime = 0;
    
    // Timing constants
    private static final double AIM_TIMEOUT = 3.0; // seconds
    private static final double SHOOTING_TIMEOUT = 5.0; // seconds
    private static final double ALIGNMENT_TOLERANCE = 5.0; // degrees
    
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
        // Simple auto shoot command initialized
        isAiming = false;
        isShooting = false;
        aimStartTime = 0;
    }
    
    @Override
    public void execute() {
        // Get current target status from centralized Limelight
        boolean hasTarget = limelightSubsystem.hasTarget();
        double horizontalOffset = limelightSubsystem.getHorizontalOffset();
        double targetArea = limelightSubsystem.getTargetArea();
        
        if (!hasTarget) {
            // No target - stop shooting and wait
            if (isShooting) {
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
            performAiming(horizontalOffset, targetArea);
        } else if (isShooting) {
            // Continue shooting
            performShooting();
        }
    }
    
    /**
     * Starts the aiming process.
     */
    private void startAiming() {
        isAiming = true;
        aimStartTime = System.currentTimeMillis() / 1000.0;
    }
    
    /**
     * Performs the aiming process.
     * 
     * @param horizontalOffset Horizontal offset to target
     * @param targetArea Target area for distance estimation
     */
    private void performAiming(double horizontalOffset, double targetArea) {
        double aimDuration = (System.currentTimeMillis() / 1000.0) - aimStartTime;
        
        // Check for timeout
        if (aimDuration > AIM_TIMEOUT) {
            isAiming = false;
            return;
        }
        
        // Check if aligned (within tolerance)
        if (Math.abs(horizontalOffset) <= ALIGNMENT_TOLERANCE) {
            // Aligned - start shooting
            isAiming = false;
            isShooting = true;
            shooterSubsystem.startPulsing();
            shootingStartTime = System.currentTimeMillis() / 1000.0;
            return;
        }
        
        // Continue aiming - drive to align with target
        double driveCorrection = calculateDriveCorrection(horizontalOffset);
        // Use driveSubsystem's drive command method
        driveSubsystem.driveArcade(() -> 0.0, () -> driveCorrection).execute();
    }
    
    /**
     * Calculates drive correction for aiming.
     * 
     * @param horizontalOffset Horizontal offset to target
     * @return Drive correction value
     */
    private double calculateDriveCorrection(double horizontalOffset) {
        // Simple proportional control
        double kP = 0.1; // Proportional gain
        double correction = horizontalOffset * kP;
        
        // Limit to reasonable voltage range
        return Math.max(-3.0, Math.min(3.0, correction));
    }
    
    /**
     * Performs the shooting process.
     */
    private void performShooting() {
        // Check if target is still valid and aligned
        if (!limelightSubsystem.hasTarget() || Math.abs(limelightSubsystem.getHorizontalOffset()) > 10.0) {
            // Target lost - stop shooting
            shooterSubsystem.stop();
            isShooting = false;
            return;
        }
        
        double shootingDuration = (System.currentTimeMillis() / 1000.0) - shootingStartTime;
        
        // Check for shooting timeout
        if (shootingDuration > SHOOTING_TIMEOUT) {
            shooterSubsystem.stop();
            isShooting = false;
            return;
        }
        
        // Continue shooting - pulsing shooter handles the rest
    }
    
    @Override
    public void end(boolean interrupted) {
        // Always stop shooter when command ends
        shooterSubsystem.stop();
        isAiming = false;
        isShooting = false;
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
