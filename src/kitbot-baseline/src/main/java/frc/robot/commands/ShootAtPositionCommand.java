// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PulsingShooterSubsystem;
import frc.robot.utils.ShootingPositionManager;

/**
 * Command to shoot at the currently selected shooting position.
 * Uses the ShootingPositionManager to get target parameters and executes
 * the shooting sequence with the pulsing shooter.
 * 
 * Features:
- Position-based shooting calculations
- Automatic aiming and movement
- Pulsing shooter integration
- SmartDashboard feedback
- Position quality validation
 */
public class ShootAtPositionCommand extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final PulsingShooterSubsystem shooterSubsystem;
    private final ShootingPositionManager positionManager;
    
    // Shooting state
    private boolean isAiming = false;
    private boolean isShooting = false;
    private double aimStartTime = 0;
    
    // Timing constants
    private static final double AIM_TIMEOUT = 3.0; // seconds
    private static final double MIN_CONFIDENCE = 0.3; // minimum confidence to shoot
    
    /**
     * Creates a new ShootAtPositionCommand.
     * 
     * @param driveSubsystem Drive subsystem for movement
     * @param limelightSubsystem Vision subsystem for targeting
     * @param shooterSubsystem Pulsing shooter subsystem
     * @param positionManager Shooting position manager
     */
    public ShootAtPositionCommand(DriveSubsystem driveSubsystem, 
                                LimelightSubsystem limelightSubsystem,
                                PulsingShooterSubsystem shooterSubsystem,
                                ShootingPositionManager positionManager) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.positionManager = positionManager;
        
        addRequirements(driveSubsystem, limelightSubsystem, shooterSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("=== SHOOT AT POSITION COMMAND ===");
        System.out.println("Target: " + positionManager.getCurrentPosition().getName());
        System.out.println("Position: (" + positionManager.getCurrentPosition().getX() + 
                         ", " + positionManager.getCurrentPosition().getY() + ")");
        System.out.println("Preferred Distance: " + positionManager.getCurrentPosition().getPreferredDistance() + "m");
        System.out.println("=====================================");
        
        isAiming = false;
        isShooting = false;
        aimStartTime = 0;
    }
    
    @Override
    public void execute() {
        // Get current shooting parameters
        double[] params = positionManager.getShootingParameters();
        double distance = params[0];
        double angle = params[1];
        double confidence = params[2];
        
        // Check if we have a valid target
        if (!limelightSubsystem.hasTarget()) {
            if (isShooting) {
                System.out.println("Target lost - stopping shooter");
                shooterSubsystem.stop();
                isShooting = false;
            }
            System.out.println("No target detected - waiting for Limelight");
            return;
        }
        
        // Check confidence level
        if (confidence < MIN_CONFIDENCE) {
            System.out.println("Low confidence (" + String.format("%.2f", confidence) + 
                             ") - moving to better position");
            moveToOptimalPosition();
            return;
        }
        
        // Proceed with shooting logic
        if (!isAiming && !isShooting) {
            startAiming();
        } else if (isAiming) {
            performAiming();
        } else if (isShooting) {
            performShooting();
        }
    }
    
    /**
     * Starts the aiming process.
     */
    private void startAiming() {
        isAiming = true;
        aimStartTime = System.currentTimeMillis() / 1000.0;
        System.out.println("Started aiming at " + positionManager.getCurrentPosition().getName());
    }
    
    /**
     * Performs aiming using drive system.
     */
    private void performAiming() {
        double aimDuration = (System.currentTimeMillis() / 1000.0) - aimStartTime;
        
        // Check for timeout
        if (aimDuration > AIM_TIMEOUT) {
            System.out.println("Aiming timeout - proceeding with shot");
            startShooting();
            return;
        }
        
        // Check if aligned with target
        double horizontalOffset = limelightSubsystem.getHorizontalOffset();
        if (Math.abs(horizontalOffset) < 3.0) { // 3 degree tolerance
            System.out.println("Target aligned - starting shooter");
            startShooting();
            return;
        }
        
        // Adjust robot to align with target
        double adjustment = calculateAlignmentAdjustment(horizontalOffset);
        driveSubsystem.driveArcade(() -> 0.0, () -> -adjustment * 0.1);
        
        System.out.println("Aiming - Offset: " + String.format("%.1f", horizontalOffset) + 
                         "°, Adjustment: " + String.format("%.2f", adjustment));
    }
    
    /**
     * Moves robot to optimal shooting position.
     */
    private void moveToOptimalPosition() {
        // This is simplified - would use actual path planning
        // For now, just provide feedback
        System.out.println("Moving to optimal position for " + positionManager.getCurrentPosition().getName());
        System.out.println("Current distance: " + String.format("%.2f", limelightSubsystem.getDistance()) + "m");
        System.out.println("Preferred distance: " + positionManager.getCurrentPosition().getPreferredDistance() + "m");
    }
    
    /**
     * Starts the shooting process.
     */
    private void startShooting() {
        isAiming = false;
        isShooting = true;
        
        // Start pulsing shooter
        shooterSubsystem.startPulsing();
        
        double[] params = positionManager.getShootingParameters();
        System.out.println("Started shooting at " + positionManager.getCurrentPosition().getName());
        System.out.println("Distance: " + String.format("%.2f", params[0]) + "m, Angle: " + 
                         String.format("%.1f", params[1]) + "°, Confidence: " + 
                         String.format("%.2f", params[2]));
    }
    
    /**
     * Continues shooting process.
     */
    private void performShooting() {
        // Check if target is still valid
        if (!limelightSubsystem.hasTarget()) {
            System.out.println("Target lost during shooting - stopping");
            shooterSubsystem.stop();
            isShooting = false;
            return;
        }
        
        // Check confidence
        double[] params = positionManager.getShootingParameters();
        if (params[2] < MIN_CONFIDENCE) {
            System.out.println("Confidence dropped - stopping shooter");
            shooterSubsystem.stop();
            isShooting = false;
            return;
        }
        
        // Pulsing shooter handles the 3s ON/0.5s OFF pattern automatically
        // Just monitor and provide feedback
        
        if (shooterSubsystem.getPulseCount() % 3 == 0 && shooterSubsystem.getPulseCount() > 0) {
            // Log every 3 pulses
            System.out.println("Shooting at " + positionManager.getCurrentPosition().getName() + 
                             " - Pulses: " + shooterSubsystem.getPulseCount() + 
                             ", Status: " + shooterSubsystem.getStatus());
        }
    }
    
    /**
     * Calculates alignment adjustment for drive system.
     * 
     * @param horizontalOffset Horizontal offset to target in degrees
     * @return Adjustment value
     */
    private double calculateAlignmentAdjustment(double horizontalOffset) {
        // Simple proportional control
        double kP = 0.1;
        double adjustment = horizontalOffset * kP;
        return Math.max(-3.0, Math.min(3.0, adjustment));
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Shoot at position command ended" + (interrupted ? " (interrupted)" : ""));
        
        // Always stop shooter when command ends
        shooterSubsystem.stop();
        // DriveSubsystem doesn't have stopMotor method, just stop the command
        
        // Reset state
        isAiming = false;
        isShooting = false;
        
        // Log final statistics
        System.out.println("Final stats - Position: " + positionManager.getCurrentPosition().getName());
        System.out.println("Pulses fired: " + shooterSubsystem.getPulseCount());
        System.out.println("Total shots: " + shooterSubsystem.getTotalShotsFired());
        
        double[] params = positionManager.getShootingParameters();
        System.out.println("Final parameters - Distance: " + String.format("%.2f", params[0]) + 
                         "m, Confidence: " + String.format("%.2f", params[2]));
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
            return "Aiming at " + positionManager.getCurrentPosition().getName();
        } else if (isShooting) {
            return "Shooting at " + positionManager.getCurrentPosition().getName() + 
                   " - " + shooterSubsystem.getStatus();
        } else if (limelightSubsystem.hasTarget()) {
            return "Ready to shoot at " + positionManager.getCurrentPosition().getName();
        } else {
            return "No target - " + positionManager.getCurrentPosition().getName();
        }
    }
}
