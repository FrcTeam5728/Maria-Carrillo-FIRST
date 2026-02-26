// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.vision.constraints.RotationConstraint;
import frc.robot.Constants.FuelConstants;

/**
 * Command that aims at the nearest AprilTag and shoots at appropriate RPM.
 * 
 * This command:
 * 1. Finds nearest AprilTag and applies rotation constraint to point towards it
 * 2. Gets distance from AprilTag
 * 3. Calculates appropriate RPM using shooting calculator
 * 4. Spins can IDs 21 and 23 to that RPM
 */
public class AprilTagAimAndShootCommand extends Command {
  private final AprilTagSubsystem aprilTagSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final FuelSubsystem fuelSubsystem;
  private final RotationConstraint rotationConstraint;
  private final ShootingCalculator shootingCalculator;

  private boolean isAimed = false;
  private boolean isAtTargetRPM = false;
  private double targetRPM = 0.0;
  private static final double AIM_TOLERANCE_DEGREES = 3.0;
  private static final double RPM_TOLERANCE = 50.0;

  /**
   * Creates a new AprilTagAimAndShootCommand.
   * 
   * @param aprilTagSubsystem The AprilTag subsystem for targeting
   * @param driveSubsystem The drive subsystem for rotation
   * @param fuelSubsystem The fuel subsystem for shooting
   */
  public AprilTagAimAndShootCommand(
      AprilTagSubsystem aprilTagSubsystem,
      DriveSubsystem driveSubsystem,
      FuelSubsystem fuelSubsystem) {
    this.aprilTagSubsystem = aprilTagSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.fuelSubsystem = fuelSubsystem;
    this.rotationConstraint = new RotationConstraint(0.1, 0.0, 0.05); // P, I, D gains
    this.shootingCalculator = new ShootingCalculator();
    
    addRequirements(aprilTagSubsystem, driveSubsystem, fuelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset aim and shoot states
    isAimed = false;
    isAtTargetRPM = false;
    
    // Enable rotation constraint
    rotationConstraint.setActive(true);
    aprilTagSubsystem.getConstraintManager().getConstraint("rotation").ifPresent(constraint -> constraint.setActive(true));
    
    System.out.println("AprilTagAimAndShootCommand: Starting aim and shoot sequence");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Step 1: Check if we have an AprilTag target
    if (!aprilTagSubsystem.hasTarget()) {
      // No target, stop motors
      driveSubsystem.stop();
      fuelSubsystem.stop();
      return;
    }

    // Step 2: Apply rotation constraint to point towards AprilTag
    var targetSpeeds = aprilTagSubsystem.getTargetSpeeds();
    if (targetSpeeds.isPresent()) {
      // Apply rotation to drive subsystem
      double rotationSpeed = targetSpeeds.get().omegaRadiansPerSecond;
      driveSubsystem.arcadeDrive(0.0, rotationSpeed);
      
      // Check if we're aimed within tolerance
      double currentYaw = aprilTagSubsystem.getTargetX(); // tx value in degrees
      if (Math.abs(currentYaw) <= AIM_TOLERANCE_DEGREES) {
        isAimed = true;
        System.out.println("AprilTagAimAndShootCommand: Target acquired, yaw = " + currentYaw + "°");
      }
    }

    // Step 3: Get distance from AprilTag
    double distance = aprilTagSubsystem.getDistanceToTarget();
    
    // Step 4: Calculate appropriate RPM using shooting calculator
    targetRPM = shootingCalculator.calculateRPM(distance);
    
    // Step 5: Spin can IDs 21 and 23 to that RPM
    fuelSubsystem.setShooterRPM(targetRPM);
    
    // Check if shooter is at target RPM
    double currentRPM = fuelSubsystem.getShooterRPM();
    if (Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE) {
      isAtTargetRPM = true;
      System.out.println("AprilTagAimAndShootCommand: Shooter at target RPM: " + currentRPM);
    }

    // If aimed and at target RPM, shoot
    if (isAimed && isAtTargetRPM) {
      fuelSubsystem.launch();
      System.out.println("AprilTagAimAndShootCommand: Shooting!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all motors
    driveSubsystem.stop();
    fuelSubsystem.stop();
    
    // Disable rotation constraint
    rotationConstraint.setActive(false);
    aprilTagSubsystem.getConstraintManager().getConstraint("rotation").ifPresent(constraint -> constraint.setActive(false));
    
    System.out.println("AprilTagAimAndShootCommand: Ended, interrupted = " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command finishes after shooting once
    return isAimed && isAtTargetRPM && fuelSubsystem.isLaunching();
  }

  /**
   * Utility class for calculating shooting RPM based on distance.
   * This would contain your team's shooting physics and calibration data.
   */
  public static class ShootingCalculator {
    // These values should be calibrated for your robot
    private static final double MIN_RPM = 1000.0;
    private static final double MAX_RPM = 6000.0;
    private static final double MIN_DISTANCE = 1.0; // meters
    private static final double MAX_DISTANCE = 8.0; // meters
    
    /**
     * Calculates the appropriate shooter RPM for a given distance.
     * 
     * @param distance Distance to target in meters
     * @return Required RPM for shooting
     */
    public double calculateRPM(double distance) {
      // Clamp distance to valid range
      distance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distance));
      
      // Linear interpolation between min and max RPM
      // This should be replaced with your actual shooting curve
      double ratio = (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
      double rpm = MIN_RPM + ratio * (MAX_RPM - MIN_RPM);
      
      System.out.println("ShootingCalculator: Distance = " + distance + "m, RPM = " + rpm);
      return rpm;
    }
    
    /**
     * Sets the shooter RPM on the fuel subsystem.
     * This assumes can IDs 21 and 23 are the shooter motors.
     */
    private void setShooterRPM(double rpm) {
      // This would need to be implemented in FuelSubsystem
      // For now, we'll use existing spin up method as approximation
      fuelSubsystem.spinUp();
    }
  }
}
