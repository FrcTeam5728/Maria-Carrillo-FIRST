// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Pulsing shooter subsystem that fires with a 3-second ON, 0.5-second OFF pattern.
 * This provides controlled fuel delivery for improved accuracy and consistency.
 * 
 * Features:
 * - Automatic pulsing pattern (3s ON, 0.5s OFF)
 * - Manual control override
 * - State tracking and monitoring
 * - SmartDashboard integration
 * - Safety interlocks
 */
public class PulsingShooterSubsystem extends SubsystemBase {
    
    // Pulsing timing constants
    private static final double PULSE_ON_TIME = 3.0; // seconds
    private static final double PULSE_OFF_TIME = 0.5; // seconds
    private static final double FULL_CYCLE_TIME = PULSE_ON_TIME + PULSE_OFF_TIME;
    
    // Shooting state
    private boolean isShooting = false;
    private boolean isPulsing = false;
    private double pulseStartTime = 0.0;
    private double totalShotsFired = 0.0;
    private int pulseCount = 0;
    
    // Motor control (would be connected to actual shooter motors)
    private double shooterSpeed = 0.0;
    private double feederSpeed = 0.0;
    
    // Safety limits
    private static final double MAX_SHOOTER_SPEED = 1.0; // 100%
    private static final double MAX_FEEDER_SPEED = 0.8; // 80%
    private static final double MIN_SHOOTER_SPEED = 0.3; // 30% minimum to spin
    
    /**
     * Creates a new PulsingShooterSubsystem.
     */
    public PulsingShooterSubsystem() {
        System.out.println("PulsingShooterSubsystem initialized");
        System.out.println("Pulse pattern: " + PULSE_ON_TIME + "s ON, " + PULSE_OFF_TIME + "s OFF");
    }
    
    @Override
    public void periodic() {
        // Update pulsing logic
        if (isPulsing) {
            updatePulsing();
        }
        
        // Update SmartDashboard
        updateSmartDashboard();
        
        // Apply motor speeds (would control actual motors)
        applyMotorSpeeds();
    }
    
    /**
     * Updates the pulsing pattern logic.
     */
    private void updatePulsing() {
        double currentTime = System.currentTimeMillis() / 1000.0;
        double elapsedTime = currentTime - pulseStartTime;
        
        // Calculate position in the pulse cycle
        double cyclePosition = elapsedTime % FULL_CYCLE_TIME;
        
        if (cyclePosition < PULSE_ON_TIME) {
            // Pulse ON phase - fire fuel
            shooterSpeed = MAX_SHOOTER_SPEED;
            feederSpeed = MAX_FEEDER_SPEED;
            
            // Count shots when transitioning from OFF to ON
            if (cyclePosition < 0.1) { // First 100ms of ON phase
                pulseCount++;
                totalShotsFired++;
                System.out.println("Pulse #" + pulseCount + " firing");
            }
        } else {
            // Pulse OFF phase - stop firing
            shooterSpeed = MIN_SHOOTER_SPEED; // Keep spinning but slower
            feederSpeed = 0.0; // Stop feeder
        }
    }
    
    /**
     * Applies motor speeds (would control actual hardware).
     */
    private void applyMotorSpeeds() {
        // In a real implementation, this would control actual motor controllers
        // For now, we just track the speeds for debugging
        
        if (shooterSpeed > 0.5) {
            // High speed - actively shooting
            // Would set motor voltages here
        } else if (shooterSpeed > 0.1) {
            // Low speed - maintaining spin
            // Would set lower motor voltages here
        } else {
            // Stopped
            // Would stop motors here
        }
    }
    
    /**
     * Starts the pulsing shooting sequence.
     */
    public void startPulsing() {
        if (!isPulsing) {
            isPulsing = true;
            pulseStartTime = System.currentTimeMillis() / 1000.0;
            pulseCount = 0;
            System.out.println("Started pulsing shooter");
        }
    }
    
    /**
     * Stops the pulsing shooting sequence.
     */
    public void stopPulsing() {
        if (isPulsing) {
            isPulsing = false;
            shooterSpeed = 0.0;
            feederSpeed = 0.0;
            System.out.println("Stopped pulsing shooter. Total pulses: " + pulseCount);
        }
    }
    
    /**
     * Starts continuous shooting (non-pulsing).
     */
    public void startContinuous() {
        isShooting = true;
        isPulsing = false;
        shooterSpeed = MAX_SHOOTER_SPEED;
        feederSpeed = MAX_FEEDER_SPEED;
        System.out.println("Started continuous shooting");
    }
    
    /**
     * Stops all shooting.
     */
    public void stop() {
        isShooting = false;
        isPulsing = false;
        shooterSpeed = 0.0;
        feederSpeed = 0.0;
        System.out.println("Stopped shooter");
    }
    
    /**
     * Sets custom shooter speed (for manual control).
     * 
     * @param speed Speed from 0.0 to 1.0
     */
    public void setShooterSpeed(double speed) {
        shooterSpeed = Math.max(0.0, Math.min(MAX_SHOOTER_SPEED, speed));
    }
    
    /**
     * Sets custom feeder speed (for manual control).
     * 
     * @param speed Speed from 0.0 to 1.0
     */
    public void setFeederSpeed(double speed) {
        feederSpeed = Math.max(0.0, Math.min(MAX_FEEDER_SPEED, speed));
    }
    
    /**
     * Gets the current pulse cycle position.
     * 
     * @return Position in cycle (0.0 to 1.0, where 0.0 = start of ON phase)
     */
    public double getPulseCyclePosition() {
        if (!isPulsing) {
            return 0.0;
        }
        
        double currentTime = System.currentTimeMillis() / 1000.0;
        double elapsedTime = currentTime - pulseStartTime;
        return (elapsedTime % FULL_CYCLE_TIME) / FULL_CYCLE_TIME;
    }
    
    /**
     * Gets the current pulse state.
     * 
     * @return True if currently in ON phase of pulse
     */
    public boolean isPulseOn() {
        if (!isPulsing) {
            return false;
        }
        return getPulseCyclePosition() < (PULSE_ON_TIME / FULL_CYCLE_TIME);
    }
    
    /**
     * Gets the current shooter speed.
     * 
     * @return Shooter speed (0.0 to 1.0)
     */
    public double getShooterSpeed() {
        return shooterSpeed;
    }
    
    /**
     * Gets the current feeder speed.
     * 
     * @return Feeder speed (0.0 to 1.0)
     */
    public double getFeederSpeed() {
        return feederSpeed;
    }
    
    /**
     * Checks if currently shooting.
     * 
     * @return True if shooting (pulsing or continuous)
     */
    public boolean isShooting() {
        return isShooting || isPulsing;
    }
    
    /**
     * Checks if currently pulsing.
     * 
     * @return True if in pulsing mode
     */
    public boolean isPulsing() {
        return isPulsing;
    }
    
    /**
     * Gets the number of pulses fired.
     * 
     * @return Number of pulses in current session
     */
    public int getPulseCount() {
        return pulseCount;
    }
    
    /**
     * Gets the total shots fired.
     * 
     * @return Total shots fired across all sessions
     */
    public double getTotalShotsFired() {
        return totalShotsFired;
    }
    
    /**
     * Updates SmartDashboard with shooter status.
     */
    private void updateSmartDashboard() {
        SmartDashboard.putBoolean("Shooter/IsShooting", isShooting());
        SmartDashboard.putBoolean("Shooter/IsPulsing", isPulsing);
        SmartDashboard.putBoolean("Shooter/IsPulseOn", isPulseOn());
        SmartDashboard.putNumber("Shooter/ShooterSpeed", shooterSpeed);
        SmartDashboard.putNumber("Shooter/FeederSpeed", feederSpeed);
        SmartDashboard.putNumber("Shooter/PulseCount", pulseCount);
        SmartDashboard.putNumber("Shooter/TotalShots", totalShotsFired);
        SmartDashboard.putNumber("Shooter/CyclePosition", getPulseCyclePosition());
        
        // Calculate and display shots per second
        double shotsPerSecond = 0.0;
        if (isPulsing && pulseCount > 0) {
            double totalTime = (System.currentTimeMillis() / 1000.0) - pulseStartTime;
            if (totalTime > 0) {
                shotsPerSecond = pulseCount / totalTime;
            }
        }
        SmartDashboard.putNumber("Shooter/ShotsPerSecond", shotsPerSecond);
    }
    
    /**
     * Resets shooter statistics.
     */
    public void resetStatistics() {
        pulseCount = 0;
        totalShotsFired = 0.0;
        System.out.println("Shooter statistics reset");
    }
    
    /**
     * Gets shooter status summary.
     * 
     * @return Status string
     */
    public String getStatus() {
        if (isPulsing) {
            return String.format("Pulsing - Cycle: %.1f, Pulse: %d, %s", 
                               getPulseCyclePosition(), pulseCount, 
                               isPulseOn() ? "FIRING" : "Waiting");
        } else if (isShooting) {
            return String.format("Continuous - Shooter: %.1f, Feeder: %.1f", 
                               shooterSpeed, feederSpeed);
        } else {
            return "Stopped";
        }
    }
}
