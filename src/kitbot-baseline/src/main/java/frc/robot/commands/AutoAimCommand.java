// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PulsingShooterSubsystem;
import frc.robot.utils.FieldPositionSystem;

/**
 * Command that automatically aims robot at AprilTag targets.
 * Combines position updates with smooth aiming and shooting control.
 * Includes D-pad target selection for POI targets.
 */
public class AutoAimCommand extends Command {
    
    private final LimelightSubsystem limelightSubsystem;
    private final FieldPositionSystem fieldPositionSystem;
    private final DriveSubsystem driveSubsystem;
    private final PulsingShooterSubsystem shooterSubsystem;
    
    // Aiming parameters
    private static final double AIM_TOLERANCE = 2.0; // degrees
    private static final double AIM_SPEED = 0.3; // rotation speed
    private static final double MIN_TARGET_AREA = 0.5; // minimum area to aim
    private static final double MAX_AIM_TIME = 3.0; // seconds timeout
    
    // State tracking
    private boolean isAiming = false;
    private double aimStartTime = 0.0;
    private int lastTargetId = -1;
    
    // Target selection with D-pad
    private int selectedTargetId = -1;
    private static final int[] POI_TARGET_IDS = {19, 20}; // Charging station IDs
    
    /**
     * Creates a new AutoAimCommand.
     * 
     * @param limelightSubsystem Limelight for target detection
     * @param fieldPositionSystem Field position system
     */
    public AutoAimCommand(LimelightSubsystem limelightSubsystem, 
                          FieldPositionSystem fieldPositionSystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.fieldPositionSystem = fieldPositionSystem;
        this.driveSubsystem = null; // Will be set if available
        this.shooterSubsystem = null; // Will be set if available
        
        addRequirements(limelightSubsystem);
    }
    
    /**
     * Creates a new AutoAimCommand with drive and shooter.
     * 
     * @param limelightSubsystem Limelight for target detection
     * @param fieldPositionSystem Field position system
     * @param driveSubsystem Drive for robot movement
     * @param shooterSubsystem Shooter for automatic shooting
     */
    public AutoAimCommand(LimelightSubsystem limelightSubsystem, 
                          FieldPositionSystem fieldPositionSystem,
                          DriveSubsystem driveSubsystem,
                          PulsingShooterSubsystem shooterSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.fieldPositionSystem = fieldPositionSystem;
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(limelightSubsystem, driveSubsystem, shooterSubsystem);
    }
    
    /**
     * Selects next POI target using D-pad.
     */
    public void selectNextTarget() {
        for (int i = 0; i < POI_TARGET_IDS.length; i++) {
            if (POI_TARGET_IDS[i] > selectedTargetId) {
                selectedTargetId = POI_TARGET_IDS[i];
                return;
            }
        }
        // Wrap around to beginning
        selectedTargetId = POI_TARGET_IDS[0];
    }
    
    /**
     * Selects previous POI target using D-pad.
     */
    public void selectPreviousTarget() {
        for (int i = POI_TARGET_IDS.length - 1; i >= 0; i--) {
            if (POI_TARGET_IDS[i] < selectedTargetId && POI_TARGET_IDS[i] > -1) {
                selectedTargetId = POI_TARGET_IDS[i];
                return;
            }
        }
        // Wrap around to end
        selectedTargetId = POI_TARGET_IDS[POI_TARGET_IDS.length - 1];
    }
    
    /**
     * Gets the currently selected target ID.
     * 
     * @return Selected target ID, or -1 if none
     */
    public int getSelectedTargetId() {
        return selectedTargetId;
    }
    
    @Override
    public void initialize() {
        isAiming = false;
        aimStartTime = System.currentTimeMillis() / 1000.0;
        lastTargetId = -1;
    }
    
    @Override
    public void execute() {
        // Check if we have a valid target
        if (!limelightSubsystem.hasTarget()) {
            isAiming = false;
            if (shooterSubsystem != null) {
                shooterSubsystem.stop();
            }
            return;
        }
        
        // Check target area and ID
        double targetArea = limelightSubsystem.getTargetArea();
        int currentTargetId = limelightSubsystem.getTargetId();
        double horizontalOffset = limelightSubsystem.getHorizontalOffset();
        double verticalOffset = limelightSubsystem.getVerticalOffset();
        
        // Use targetArea in validation
        if (targetArea < MIN_TARGET_AREA || currentTargetId <= 0) {
            isAiming = false;
            return;
        }
        
        // Check if this is a POI target
        if (isPOITarget(currentTargetId)) {
            // Only aim at POI targets
            performPOIAiming(currentTargetId, horizontalOffset, verticalOffset);
        } else {
            // Regular AprilTag aiming
            performRegularAiming(currentTargetId, horizontalOffset, verticalOffset);
        }
    }
    
    /**
     * Checks if target ID corresponds to a POI.
     * 
     * @param targetId AprilTag ID to check
     * @return True if this is a POI target
     */
    private boolean isPOITarget(int targetId) {
        // POI targets from JSON file
        int[] poiTargetIds = {19, 20}; // Charging station IDs
        
        for (int poiId : poiTargetIds) {
            if (targetId == poiId) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * Performs aiming at POI targets.
     */
    private void performPOIAiming(int targetId, double horizontalOffset, double verticalOffset) {
        // Priority aiming at important POIs
        if (driveSubsystem != null) {
            double currentHeading = fieldPositionSystem.getRobotPose().getRotation().getDegrees();
            double newHeading = currentHeading + (calculateRotationCorrection(horizontalOffset, verticalOffset) * AIM_SPEED);
            
            // Create smooth rotation command using driveArcade with proper suppliers
            driveSubsystem.driveArcade(
                () -> 0.0, // No forward movement
                () -> newHeading // Only rotation
            );
        }
    }
    
    /**
     * Performs regular AprilTag aiming.
     */
    private void performRegularAiming(int targetId, double horizontalOffset, double verticalOffset) {
        // Standard aiming logic for non-POI AprilTags
        // Check if we have a valid target
        if (!limelightSubsystem.hasTarget()) {
            isAiming = false;
            return;
        }
        
        // Check target area and ID
        double targetArea = limelightSubsystem.getTargetArea();
        int currentTargetId = limelightSubsystem.getTargetId();
        
        // Only aim at substantial targets
        if (targetArea < MIN_TARGET_AREA || currentTargetId <= 0) {
            isAiming = false;
            return;
        }
        
        // Check for target change
        if (currentTargetId != lastTargetId) {
            lastTargetId = currentTargetId;
            isAiming = true;
            aimStartTime = System.currentTimeMillis() / 1000.0;
        }
        
        // Perform aiming if active
        if (isAiming) {
            performAiming();
        }
    }
    
    /**
     * Calculates the rotation needed to center target.
     * 
     * @param horizontalOffset Horizontal offset in degrees
     * @param verticalOffset Vertical offset in degrees
     * @return Required rotation correction
     */
    private double calculateRotationCorrection(double horizontalOffset, double verticalOffset) {
        // Simple proportional control with deadband
        if (Math.abs(horizontalOffset) < AIM_TOLERANCE) {
            return 0.0; // Within tolerance, no correction needed
        }
        
        // Consider both horizontal and vertical offset for more accurate aiming
        double combinedOffset = Math.sqrt(horizontalOffset * horizontalOffset + verticalOffset * verticalOffset);
        
        // Proportional correction with scaling
        return -combinedOffset * 0.05; // Smaller scale for gentler corrections
    }
    
    /**
     * Performs the actual aiming logic.
     */
    private void performAiming() {
        double horizontalOffset = limelightSubsystem.getHorizontalOffset();
        double verticalOffset = limelightSubsystem.getVerticalOffset();
        
        // Calculate required rotation to center target
        double rotationCorrection = calculateRotationCorrection(horizontalOffset, verticalOffset);
        
        // Apply smooth rotation correction
        if (driveSubsystem != null) {
            double currentHeading = fieldPositionSystem.getRobotPose().getRotation().getDegrees();
            double newHeading = currentHeading + (rotationCorrection * AIM_SPEED);
            
            // Create smooth rotation command using driveArcade with proper suppliers
            driveSubsystem.driveArcade(
                () -> 0.0, // No forward movement
                () -> newHeading // Only rotation
            );
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        isAiming = false;
        
        // Stop any ongoing movements
        if (driveSubsystem != null) {
            driveSubsystem.driveArcade(0.0, 0.0);
        }
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes when aimed or timeout
        return !isAiming || 
               (System.currentTimeMillis() / 1000.0 - aimStartTime > MAX_AIM_TIME);
    }
    
    /**
     * Gets current aiming status.
     * 
     * @return True if currently aiming
     */
    public boolean isAiming() {
        return isAiming;
    }
    
    /**
     * Gets current target ID.
     * 
     * @return Current target ID, or -1 if none
     */
    public int getCurrentTargetId() {
        return lastTargetId;
    }
    
    /**
     * Gets horizontal offset to target.
     * 
     * @return Horizontal offset in degrees
     */
    public double getHorizontalOffset() {
        return limelightSubsystem.getHorizontalOffset();
    }
    
    /**
     * Gets target area.
     * 
     * @return Target area (0-100)
     */
    public double getTargetArea() {
        return limelightSubsystem.getTargetArea();
    }
}
