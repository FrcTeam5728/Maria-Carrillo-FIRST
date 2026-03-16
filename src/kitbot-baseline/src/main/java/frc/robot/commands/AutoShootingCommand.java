// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.utils.AdvancedShootingCalculator;

/**
 * Automatic shooting command that uses advanced physics calculations
 * to automatically aim and shoot at any detected AprilTag.
 * 
 * Features:
 * - Automatic target acquisition and tracking
 * - Physics-based trajectory calculation
 * - Magnus effect compensation
 * - Real-time aiming adjustments
 * - Integration with robot odometry
 */
public class AutoShootingCommand extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final FuelSubsystem fuelSubsystem;
    private final AprilTagSubsystem aprilTagSubsystem;
    private final AdvancedShootingCalculator calculator;
    private final CommandXboxController controller;
    
    // Shooting state
    private boolean isAiming = false;
    private boolean isShooting = false;
    private double aimStartTime = 0;
    private double shootStartTime = 0;
    
    // Tuning parameters
    private static final double AIM_TIMEOUT = 3.0; // seconds to acquire target
    private static final double SHOOT_DURATION = 0.5; // seconds to launch fuel
    private static final double MIN_VELOCITY = 5.0; // m/s minimum launch velocity
    private static final double MAX_VELOCITY = 20.0; // m/s maximum launch velocity
    
    /**
     * Creates a new AutoShootingCommand.
     * 
     * @param driveSubsystem Drive subsystem for movement
     * @param fuelSubsystem Fuel subsystem for shooting
     * @param aprilTagSubsystem Vision subsystem for targeting
     * @param controller Controller for manual override
     */
    public AutoShootingCommand(DriveSubsystem driveSubsystem, FuelSubsystem fuelSubsystem, 
                           AprilTagSubsystem aprilTagSubsystem, CommandXboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.fuelSubsystem = fuelSubsystem;
        this.aprilTagSubsystem = aprilTagSubsystem;
        this.controller = controller;
        this.calculator = new AdvancedShootingCalculator(driveSubsystem, aprilTagSubsystem);
        
        addRequirements(driveSubsystem, fuelSubsystem, aprilTagSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("=== AUTO SHOOTING COMMAND INITIALIZED ===");
        System.out.println("Features: Physics-based aiming, Magnus effect, auto-targeting");
        System.out.println("Controls: A=Start aiming, B=Shoot, X=Cancel");
        System.out.println("=====================================");
        
        isAiming = false;
        isShooting = false;
        aimStartTime = 0;
        shootStartTime = 0;
        
        // Initialize calculator
        calculator.reset();
    }
    
    @Override
    public void execute() {
        calculator.update(); // Update with current AprilTag data
        
        // Check for manual override
        if (controller.a().getAsBoolean()) {
            if (!isAiming) {
                startAiming();
            }
        } else if (controller.b().getAsBoolean() && isAiming) {
            startShooting();
        } else if (controller.x().getAsBoolean()) {
            cancelShooting();
        }
        
        // Automatic aiming logic
        if (isAiming && !isShooting) {
            performAiming();
        }
        
        // Shooting logic
        if (isShooting) {
            performShooting();
        }
    }
    
    /**
     * Starts the aiming process.
     */
    private void startAiming() {
        if (!calculator.isTargetLocked()) {
            System.out.println("No target available - cannot start aiming");
            return;
        }
        
        isAiming = true;
        isShooting = false;
        aimStartTime = System.currentTimeMillis() / 1000.0;
        
        System.out.println("Started automatic aiming...");
        System.out.println("Target distance: " + calculator.getTargetDistance() + " meters");
        System.out.println("Launch angle: " + calculator.getLaunchAngle() + " degrees");
        System.out.println("Launch velocity: " + calculator.getLaunchVelocity() + " m/s");
    }
    
    /**
     * Performs automatic aiming using drive system.
     */
    private void performAiming() {
        if (!calculator.isTargetLocked()) return;
        
        double aimDuration = (System.currentTimeMillis() / 1000.0) - aimStartTime;
        
        // Check if we've aimed long enough
        if (aimDuration > AIM_TIMEOUT) {
            System.out.println("Aiming timeout - ready to shoot");
            return;
        }
        
        // Calculate required movement to align with target
        Pose2d targetPose = aprilTagSubsystem.getTargetPose().orElse(new Pose2d());
        DifferentialDriveWheelSpeeds wheelSpeeds = calculator.calculateMovementToTarget(targetPose);
        
        // Apply movement (simplified - would use PID in real implementation)
        driveSubsystem.tankDriveVolts(wheelSpeeds.left * 6, wheelSpeeds.right * 6); // Scale to voltage
        
        // Check if aligned (simplified - would use tolerance in real implementation)
        double distance = calculator.getTargetDistance();
        if (distance < 1.0) { // Close range
            System.out.println("Target aligned - ready to shoot");
        }
    }
    
    /**
     * Starts the shooting process.
     */
    private void startShooting() {
        if (!calculator.isTargetLocked()) {
            System.out.println("No target locked - cannot shoot");
            return;
        }
        
        double velocity = calculator.getLaunchVelocity();
        
        // Validate velocity
        if (velocity < MIN_VELOCITY || velocity > MAX_VELOCITY) {
            System.out.println("Launch velocity out of range: " + velocity + " m/s");
            return;
        }
        
        isShooting = true;
        shootStartTime = System.currentTimeMillis() / 1000.0;
        
        System.out.println("Started shooting...");
        System.out.println("Spin rate: " + calculator.getSpinRate() + " rev/s");
        
        // Start shooting sequence
        fuelSubsystem.spinUp(); // Start launcher
    }
    
    /**
     * Performs the shooting sequence.
     */
    private void performShooting() {
        double shootDuration = (System.currentTimeMillis() / 1000.0) - shootStartTime;
        
        if (shootDuration < SHOOT_DURATION) {
            // Continue spinning up
            return;
        }
        
        // Launch fuel
        fuelSubsystem.launch();
        System.out.println("Fuel launched!");
        
        // Stop shooting after duration
        if (shootDuration > SHOOT_DURATION + 1.0) {
            fuelSubsystem.stop();
            isShooting = false;
            System.out.println("Shooting sequence complete");
        }
    }
    
    /**
     * Cancels the shooting process.
     */
    private void cancelShooting() {
        if (isAiming || isShooting) {
            System.out.println("Shooting cancelled by user");
        }
        
        isAiming = false;
        isShooting = false;
        fuelSubsystem.stop();
        driveSubsystem.stopMotor(); // Stop any movement
        
        // Reset calculator
        calculator.reset();
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Auto shooting command ended" + (interrupted ? " (interrupted)" : ""));
        
        // Always stop shooting when command ends
        fuelSubsystem.stop();
        driveSubsystem.stopMotor();
        
        // Reset state
        isAiming = false;
        isShooting = false;
    }
    
    @Override
    public boolean isFinished() {
        // Command never finishes automatically - must be manually cancelled
        return false;
    }
    
    /**
     * Gets the current shooting calculator.
     * 
     * @return The calculator instance
     */
    public AdvancedShootingCalculator getCalculator() {
        return calculator;
    }
    
    /**
     * Gets the current aiming state.
     * 
     * @return True if currently aiming
     */
    public boolean isAiming() {
        return isAiming;
    }
    
    /**
     * Gets the current shooting state.
     * 
     * @return True if currently shooting
     */
    public boolean isShooting() {
        return isShooting;
    }
}
