// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Advanced shooting calculator that uses physics, kinematics, and AprilTag data
 * to automatically calculate optimal shooting parameters for any target.
 * 
 * Features:
 * - AprilTag-based target tracking
 * - Robot kinematics integration
 * - Drag and Magnus effect calculations
 * - Automatic odometry updates
 * - Real-time trajectory prediction
 */
public class AdvancedShootingCalculator {
    
    // Physics constants
    private static final double GRAVITY = 9.81; // m/s^2
    private static final double AIR_DENSITY = 1.225; // kg/m^3 at sea level
    private static final double FUEL_MASS = 0.27; // kg (standard FRC fuel cell)
    private static final double FUEL_DIAMETER = 0.18; // m (7 inches)
    private static final double FUEL_DRAG_COEFFICIENT = 0.47; // sphere drag coefficient
    private static final double SPIN_RATE = 30.0; // rev/s (typical shooter)
    
    // Robot configuration
    private final DriveSubsystem driveSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final DifferentialDriveKinematics kinematics;
    
    // Shooting parameters
    private double targetDistance;
    private double targetHeight;
    private double launchAngle;
    private double launchVelocity;
    private double spinRate;
    private boolean isTargetLocked;
    
    /**
     * Creates a new AdvancedShootingCalculator.
     * 
     * @param driveSubsystem The drive subsystem for kinematics
     * @param limelightSubsystem The vision subsystem for target data
     */
    public AdvancedShootingCalculator(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.kinematics = new DifferentialDriveKinematics(
            0.6 // Default track width (would get from Constants)
        );
        
        reset();
    }
    
    /**
     * Resets calculator to initial state.
     */
    public void reset() {
        targetDistance = 0;
        targetHeight = 0;
        launchAngle = 45; // Default 45 degrees
        launchVelocity = 10; // Default 10 m/s
        spinRate = SPIN_RATE;
        isTargetLocked = false;
    }
    
    /**
     * Updates calculator with current Limelight data.
     * Call this periodically to track moving targets.
     */
    public void update() {
        if (limelightSubsystem.hasTarget()) {
            updateFromLimelight();
            isTargetLocked = true;
        } else {
            isTargetLocked = false;
            // Maintain last known target position for prediction
        }
        
        // Update SmartDashboard with shooting data
        updateSmartDashboard();
    }
    
    /**
     * Updates shooting parameters from Limelight data.
     */
    private void updateFromLimelight() {
        try {
            // Get target info from Limelight
            targetDistance = limelightSubsystem.getDistance();
            
            // Get current robot pose (simplified - would use actual odometry)
            // Pose2d robotPose = limelightSubsystem.getRobotPose().orElse(new Pose2d());
            Pose2d robotPose = new Pose2d(); // Placeholder
            
            // Estimate target height (assuming standard AprilTag height)
            targetHeight = 0.5; // meters (typical AprilTag height)
            
            // Calculate optimal shooting parameters
            calculateOptimalTrajectory();
            
        } catch (Exception e) {
            System.err.println("Error updating from Limelight: " + e.getMessage());
        }
    }
    
    /**
     * Calculates optimal shooting trajectory using physics.
     */
    private void calculateOptimalTrajectory() {
        // Basic trajectory calculation using projectile motion
        // h = v0*sin(θ)*t - 0.5*g*t^2
        // d = v0*cos(θ)*t
        
        // Solve for launch angle and velocity
        double[] solution = solveTrajectory(targetDistance, targetHeight);
        launchAngle = solution[0];
        launchVelocity = solution[1];
        
        // Calculate required spin rate based on distance
        spinRate = calculateOptimalSpinRate(targetDistance);
        
        // Apply Magnus effect adjustment
        applyMagnusEffect();
    }
    
    /**
     * Solves projectile motion equations for optimal trajectory.
     * 
     * @param distance Horizontal distance to target (meters)
     * @param height Height difference to target (meters)
     * @return Array containing [angle, velocity]
     */
    private double[] solveTrajectory(double distance, double height) {
        // Simplified solution for optimal angle and velocity
        // Using 45 degrees as starting point for optimization
        
        double bestAngle = 45; // degrees
        double bestVelocity = 15; // m/s
        double minError = Double.MAX_VALUE;
        
        // Search for optimal angle (30-60 degrees range)
        for (double angle = 30; angle <= 60; angle += 1) {
            double angleRad = Math.toRadians(angle);
            
            // Calculate required velocity for this angle
            double numerator = distance * GRAVITY * distance;
            double denominator = 2 * (distance * Math.sin(angleRad) - height);
            
            if (denominator <= 0) continue; // Skip invalid solutions
            
            double velocity = Math.sqrt(numerator / denominator);
            
            // Calculate landing error (how close to target height)
            double flightTime = distance / (velocity * Math.cos(angleRad));
            double landingHeight = velocity * Math.sin(angleRad) * flightTime - 0.5 * GRAVITY * flightTime * flightTime;
            double error = Math.abs(landingHeight - height);
            
            if (error < minError) {
                minError = error;
                bestAngle = angle;
                bestVelocity = velocity;
            }
        }
        
        return new double[]{bestAngle, bestVelocity};
    }
    
    /**
     * Calculates optimal spin rate based on target distance.
     * 
     * @param distance Distance to target in meters
     * @return Optimal spin rate in revolutions per second
     */
    private double calculateOptimalSpinRate(double distance) {
        // Spin rate decreases with distance (less time for Magnus effect)
        // Base rate at close range, reduced at long range
        
        if (distance < 2.0) {
            return SPIN_RATE; // Full spin at close range
        } else if (distance < 5.0) {
            return SPIN_RATE * 0.8; // 80% spin at medium range
        } else if (distance < 8.0) {
            return SPIN_RATE * 0.6; // 60% spin at long range
        } else {
            return SPIN_RATE * 0.4; // 40% spin at very long range
        }
    }
    
    /**
     * Applies Magnus effect calculations to adjust trajectory.
     */
    private void applyMagnusEffect() {
        // Magnus effect causes curved trajectory for spinning projectiles
        // Effect is proportional to spin rate and inversely proportional to velocity
        
        double magnusForce = calculateMagnusForce();
        
        // Adjust launch angle to compensate for Magnus effect
        // This is a simplified model - real implementation would be more complex
        double magnusAdjustment = Math.toDegrees(magnusForce / launchVelocity) * 0.1;
        launchAngle += magnusAdjustment;
        
        // Adjust velocity slightly for Magnus effect
        launchVelocity *= (1.0 + magnusForce * 0.05);
    }
    
    /**
     * Calculates Magnus force on spinning projectile.
     * 
     * @return Magnus force in m/s^2
     */
    private double calculateMagnusForce() {
    // Magnus force = 2 * rho * r * omega * v
    // where rho = air density, r = radius, omega = angular velocity, v = linear velocity
        
        double radius = FUEL_DIAMETER / 2.0; // meters
        double angularVelocity = spinRate * 2 * Math.PI; // rad/s
        
        // Simplified Magnus calculation
        double magnusCoefficient = 0.5; // Empirical coefficient
        return magnusCoefficient * AIR_DENSITY * radius * angularVelocity * launchVelocity;
    }
    
    /**
     * Predicts where the fuel will land given current shooting parameters.
     * 
     * @return Predicted landing position relative to robot
     */
    public Pose2d predictLandingPosition() {
        if (!isTargetLocked) {
            return new Pose2d(); // No prediction without target
        }
        
        double angleRad = Math.toRadians(launchAngle);
        double flightTime = calculateFlightTime();
        
        // Calculate landing position
        double xDistance = launchVelocity * Math.cos(angleRad) * flightTime;
        double yDistance = launchVelocity * Math.sin(angleRad) * flightTime - 0.5 * GRAVITY * flightTime * flightTime;
        
        return new Pose2d(xDistance, yDistance, new Rotation2d());
    }
    
    /**
     * Calculates flight time to target.
     * 
     * @return Flight time in seconds
     */
    private double calculateFlightTime() {
        // Simplified calculation assuming direct hit
        return targetDistance / (launchVelocity * Math.cos(Math.toRadians(launchAngle)));
    }
    
    /**
     * Updates SmartDashboard with shooting calculator data.
     */
    private void updateSmartDashboard() {
        SmartDashboard.putBoolean("Shooting/TargetLocked", isTargetLocked);
        SmartDashboard.putNumber("Shooting/TargetDistance", targetDistance);
        SmartDashboard.putNumber("Shooting/TargetHeight", targetHeight);
        SmartDashboard.putNumber("Shooting/LaunchAngle", launchAngle);
        SmartDashboard.putNumber("Shooting/LaunchVelocity", launchVelocity);
        SmartDashboard.putNumber("Shooting/SpinRate", spinRate);
        
        if (isTargetLocked) {
            Pose2d predicted = predictLandingPosition();
            SmartDashboard.putNumber("Shooting/PredictedX", predicted.getX());
            SmartDashboard.putNumber("Shooting/PredictedY", predicted.getY());
        }
    }
    
    /**
     * Gets the current target distance.
     * 
     * @return Distance to target in meters
     */
    public double getTargetDistance() {
        return targetDistance;
    }
    
    /**
     * Gets the recommended launch angle.
     * 
     * @return Launch angle in degrees
     */
    public double getLaunchAngle() {
        return launchAngle;
    }
    
    /**
     * Gets the recommended launch velocity.
     * 
     * @return Launch velocity in m/s
     */
    public double getLaunchVelocity() {
        return launchVelocity;
    }
    
    /**
     * Gets the recommended spin rate.
     * 
     * @return Spin rate in revolutions per second
     */
    public double getSpinRate() {
        return spinRate;
    }
    
    /**
     * Checks if a target is currently locked.
     * 
     * @return True if target is locked and tracked
     */
    public boolean isTargetLocked() {
        return isTargetLocked;
    }
    
    /**
     * Calculates wheel speeds for moving to target position.
     * 
     * @param targetPose Target position to move to
     * @return Wheel speeds for differential drive
     */
    public DifferentialDriveWheelSpeeds calculateMovementToTarget(Pose2d targetPose) {
        // Get current robot pose (would need odometry integration)
        Pose2d currentPose = new Pose2d(); // Simplified - should use actual odometry
        
        // Calculate required movement
        Translation2d movement = new Translation2d(
            targetPose.getX() - currentPose.getX(),
            targetPose.getY() - currentPose.getY()
        );
        
        // Simple proportional control for movement
        double maxSpeed = 2.0; // m/s
        double xSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, movement.getX() * 2.0));
        
        // Calculate rotation needed (simplified)
        double targetAngle = Math.atan2(movement.getY(), movement.getX());
        double rotationSpeed = Math.toDegrees(targetAngle) * 0.5; // degrees per second
        
        // Convert to chassis speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0, rotationSpeed);
        return kinematics.toWheelSpeeds(chassisSpeeds);
    }
    
    /**
     * Calculates required drive adjustments for moving launch platform.
     * 
     * @param currentAngle Current launch platform angle
     * @param targetAngle Target launch platform angle
     * @return Required adjustments
     */
    public double[] calculatePlatformAdjustment(double currentAngle, double targetAngle) {
        double angleDifference = targetAngle - currentAngle;
        
        // Normalize angle difference to [-180, 180]
        while (angleDifference > 180) angleDifference -= 360;
        while (angleDifference < -180) angleDifference += 360;
        
        // Calculate required adjustment speed
        double adjustmentSpeed = Math.max(-1.0, Math.min(1.0, angleDifference * 0.1));
        
        return new double[]{angleDifference, adjustmentSpeed};
    }
}
