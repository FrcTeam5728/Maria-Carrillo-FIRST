// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Shooting position manager that stores and manages predefined shooting positions.
 * Allows D-pad selection of different shooting positions on the field.
 * 
 * Features:
 * - Predefined shooting positions (Speaker, Amp, Stage, etc.)
 * - D-pad navigation through positions
 * - Distance-based shooting calculations
 * - SmartDashboard integration
 * - Position validation and safety checks
 */
public class ShootingPositionManager {
    
    // Shooting positions (field coordinates in meters)
    public enum ShootingPosition {
        SPEAKER_CENTER("Speaker Center", 8.02, 0.22, 0, 2.5),
        SPEAKER_LEFT("Speaker Left", 7.5, 0.5, -15, 2.8),
        SPEAKER_RIGHT("Speaker Right", 8.5, 0.5, 15, 2.8),
        AMP("Amp", 1.5, 7.0, -90, 1.5),
        STAGE_LEFT("Stage Left", 4.5, 2.5, -30, 4.0),
        STAGE_CENTER("Stage Center", 4.5, 4.1, 0, 4.0),
        STAGE_RIGHT("Stage Right", 4.5, 5.7, 30, 4.0),
        SOURCE_SIDE("Source Side", 15.0, 1.0, 180, 6.0),
        WING("Wing Position", 10.0, 2.0, 45, 3.5),
        PODIUM("Podium", 2.0, 4.1, 0, 2.0);
        
        private final String name;
        private final double x;
        private final double y;
        private final double rotation;
        private final double preferredDistance;
        
        ShootingPosition(String name, double x, double y, double rotation, double preferredDistance) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.rotation = rotation;
            this.preferredDistance = preferredDistance;
        }
        
        public String getName() { return name; }
        public double getX() { return x; }
        public double getY() { return y; }
        public double getRotation() { return rotation; }
        public double getPreferredDistance() { return preferredDistance; }
        
        public Pose2d getPose() {
            return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
        }
    }
    
    // Current selection
    private ShootingPosition currentPosition = ShootingPosition.SPEAKER_CENTER;
    private int positionIndex = 0;
    
    // Limelight subsystem for distance calculations
    private final LimelightSubsystem limelightSubsystem;
    
    /**
     * Creates a new ShootingPositionManager.
     * 
     * @param limelightSubsystem Limelight subsystem for distance data
     */
    public ShootingPositionManager(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        
        System.out.println("ShootingPositionManager initialized");
        System.out.println("Default position: " + currentPosition.getName());
        printAllPositions();
    }
    
    /**
     * Selects the next shooting position (D-pad right).
     */
    public void selectNextPosition() {
        positionIndex = (positionIndex + 1) % ShootingPosition.values().length;
        currentPosition = ShootingPosition.values()[positionIndex];
        System.out.println("Selected position: " + currentPosition.getName());
        updateSmartDashboard();
    }
    
    /**
     * Selects the previous shooting position (D-pad left).
     */
    public void selectPreviousPosition() {
        positionIndex = (positionIndex - 1 + ShootingPosition.values().length) % ShootingPosition.values().length;
        currentPosition = ShootingPosition.values()[positionIndex];
        System.out.println("Selected position: " + currentPosition.getName());
        updateSmartDashboard();
    }
    
    /**
     * Selects shooting position by D-pad direction.
     * 
     * @param direction D-pad direction (0=up, 90=right, 180=down, 270=left)
     */
    public void selectPositionByDirection(int direction) {
        switch (direction) {
            case 0: // Up - Speaker positions
                if (positionIndex < 3) {
                    positionIndex = (positionIndex + 1) % 3;
                } else {
                    positionIndex = 0;
                }
                break;
            case 90: // Right - Next position
                selectNextPosition();
                return;
            case 180: // Down - Stage positions
                if (positionIndex >= 4 && positionIndex <= 6) {
                    positionIndex = 4 + ((positionIndex - 4 + 1) % 3);
                } else {
                    positionIndex = 4;
                }
                break;
            case 270: // Left - Previous position
                selectPreviousPosition();
                return;
        }
        currentPosition = ShootingPosition.values()[positionIndex];
        System.out.println("Selected position: " + currentPosition.getName());
        updateSmartDashboard();
    }
    
    /**
     * Gets the current shooting position.
     * 
     * @return Current shooting position
     */
    public ShootingPosition getCurrentPosition() {
        return currentPosition;
    }
    
    /**
     * Gets the current position pose.
     * 
     * @return Current position as Pose2d
     */
    public Pose2d getCurrentPose() {
        return currentPosition.getPose();
    }
    
    /**
     * Calculates distance from current robot position to shooting position.
     * 
     * @param robotPose Current robot pose
     * @return Distance in meters
     */
    public double calculateDistanceToPosition(Pose2d robotPose) {
        double dx = currentPosition.getX() - robotPose.getX();
        double dy = currentPosition.getY() - robotPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
     * Gets shooting parameters for current position.
     * 
     * @return Shooting parameters array [distance, angle, confidence]
     */
    public double[] getShootingParameters() {
        double distance = limelightSubsystem.getDistance();
        double preferredDistance = currentPosition.getPreferredDistance();
        double confidence = 0.0;
        
        // Calculate confidence based on distance match
        if (distance > 0) {
            double distanceDiff = Math.abs(distance - preferredDistance);
            confidence = Math.max(0.0, 1.0 - (distanceDiff / preferredDistance));
        }
        
        // Calculate optimal shooting angle based on distance
        double angle = calculateOptimalAngle(distance);
        
        return new double[]{distance, angle, confidence};
    }
    
    /**
     * Calculates optimal shooting angle for given distance.
     * 
     * @param distance Distance to target in meters
     * @return Optimal angle in degrees
     */
    private double calculateOptimalAngle(double distance) {
        // Simplified angle calculation
        // In reality, this would use physics calculations
        if (distance < 2.0) return 45.0;
        if (distance < 4.0) return 35.0;
        if (distance < 6.0) return 25.0;
        return 20.0;
    }
    
    /**
     * Checks if robot is in good shooting position.
     * 
     * @param robotPose Current robot pose
     * @return True if in good position
     */
    public boolean isInGoodPosition(Pose2d robotPose) {
        double distance = calculateDistanceToPosition(robotPose);
        double preferredDistance = currentPosition.getPreferredDistance();
        double tolerance = 1.0; // 1 meter tolerance
        
        return Math.abs(distance - preferredDistance) < tolerance;
    }
    
    /**
     * Gets position quality rating.
     * 
     * @param robotPose Current robot pose
     * @return Quality string
     */
    public String getPositionQuality(Pose2d robotPose) {
        double[] params = getShootingParameters();
        double confidence = params[2];
        
        if (confidence >= 0.8) return "EXCELLENT";
        if (confidence >= 0.6) return "GOOD";
        if (confidence >= 0.4) return "FAIR";
        if (confidence >= 0.2) return "POOR";
        return "BAD";
    }
    
    /**
     * Updates SmartDashboard with current position info.
     */
    private void updateSmartDashboard() {
        SmartDashboard.putString("Shooting/CurrentPosition", currentPosition.getName());
        SmartDashboard.putNumber("Shooting/PositionX", currentPosition.getX());
        SmartDashboard.putNumber("Shooting/PositionY", currentPosition.getY());
        SmartDashboard.putNumber("Shooting/PositionRotation", currentPosition.getRotation());
        SmartDashboard.putNumber("Shooting/PreferredDistance", currentPosition.getPreferredDistance());
        
        // Update shooting parameters if Limelight has target
        if (limelightSubsystem.hasTarget()) {
            double[] params = getShootingParameters();
            SmartDashboard.putNumber("Shooting/Distance", params[0]);
            SmartDashboard.putNumber("Shooting/OptimalAngle", params[1]);
            SmartDashboard.putNumber("Shooting/Confidence", params[2]);
            SmartDashboard.putString("Shooting/Quality", getPositionQuality(new Pose2d()));
        }
    }
    
    /**
     * Updates SmartDashboard (call periodically).
     */
    public void update() {
        updateSmartDashboard();
    }
    
    /**
     * Prints all available shooting positions.
     */
    public void printAllPositions() {
        System.out.println("=== AVAILABLE SHOOTING POSITIONS ===");
        for (int i = 0; i < ShootingPosition.values().length; i++) {
            ShootingPosition pos = ShootingPosition.values()[i];
            String marker = (i == positionIndex) ? ">>> " : "    ";
            System.out.println(marker + (i + 1) + ". " + pos.getName() + 
                             " at (" + pos.getX() + ", " + pos.getY() + 
                             ") - " + pos.getPreferredDistance() + "m");
        }
        System.out.println("=====================================");
    }
    
    /**
     * Gets status summary.
     * 
     * @return Status string
     */
    public String getStatus() {
        double[] params = getShootingParameters();
        return String.format("Position: %s | Distance: %.2f | Angle: %.1f° | Quality: %s",
                           currentPosition.getName(), params[0], params[1], getPositionQuality(new Pose2d()));
    }
    
    /**
     * Resets to default position.
     */
    public void reset() {
        currentPosition = ShootingPosition.SPEAKER_CENTER;
        positionIndex = 0;
        System.out.println("Shooting position reset to: " + currentPosition.getName());
        updateSmartDashboard();
    }
}
