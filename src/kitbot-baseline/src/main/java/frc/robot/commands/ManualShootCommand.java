package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.ShootingDistanceControl;
import frc.robot.Constants.FuelConstants;

/**
 * Command that shoots using the manually set distance from ShootingDistanceControl.
 * Uses the D-pad controlled distance instead of AprilTag distance.
 */
public class ManualShootCommand extends Command {
  
  private final FuelSubsystem fuelSubsystem;
  private final ShootingDistanceControl distanceControl;
  
  private boolean isAtTargetRPM = false;
  private double spinupStartTime = 0.0;
  private double targetRPM = 0.0;
  private static final double RPM_TOLERANCE = 50.0;
  
  /**
   * Creates a new ManualShootCommand.
   * 
   * @param fuelSubsystem The fuel subsystem for shooting
   * @param distanceControl The distance control subsystem for target distance
   */
  public ManualShootCommand(FuelSubsystem fuelSubsystem, ShootingDistanceControl distanceControl) {
    this.fuelSubsystem = fuelSubsystem;
    this.distanceControl = distanceControl;
    
    addRequirements(fuelSubsystem);
  }
  
  @Override
  public void initialize() {
    // Reset state
    isAtTargetRPM = false;
    spinupStartTime = 0.0;
    
    // Get target RPM from current distance setting
    targetRPM = distanceControl.getTargetRPM();
    
    System.out.println("ManualShootCommand: Starting manual shot");
    System.out.println("  Target distance: " + String.format("%.2f", distanceControl.getCurrentDistanceFeet()) + " ft");
    System.out.println("  Target RPM: " + String.format("%.0f", targetRPM));
  }
  
  @Override
  public void execute() {
    // Set shooter to target RPM
    fuelSubsystem.setShooterRPM(targetRPM);
    
    // Start spinup timer if this is the first time
    if (spinupStartTime == 0.0) {
      spinupStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      System.out.println("DEBUG: Starting spinup timer for manual shot");
    }
    
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double spinupElapsed = currentTime - spinupStartTime;
    
    // Check if shooter is at target RPM or spinup time has elapsed
    double currentRPM = fuelSubsystem.getShooterRPM();
    boolean rpmInTolerance = Math.abs(currentRPM - targetRPM) <= RPM_TOLERANCE;
    boolean spinupTimeElapsed = spinupElapsed >= FuelConstants.SHOOTER_SPINUP_TIME_SECONDS;
    
    if (rpmInTolerance) {
      isAtTargetRPM = true;
      System.out.println("DEBUG: Manual shooter at target RPM: " + String.format("%.0f", currentRPM) + 
                         " after " + String.format("%.2f", spinupElapsed) + "s");
    } else if (spinupTimeElapsed) {
      isAtTargetRPM = true;
      System.out.println("DEBUG: Manual spinup time elapsed, proceeding with RPM: " + 
                         String.format("%.0f", currentRPM));
    } else {
      System.out.println("DEBUG: Manual spinning up - Current: " + String.format("%.0f", currentRPM) + 
                         " RPM, Target: " + String.format("%.0f", targetRPM) + 
                         " RPM, Elapsed: " + String.format("%.2f", spinupElapsed) + "s");
    }
    
    // If at target RPM, launch
    if (isAtTargetRPM) {
      fuelSubsystem.launch();
      System.out.println("ManualShootCommand: Shooting manually!");
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
    System.out.println("ManualShootCommand: Ended, interrupted = " + interrupted);
  }
  
  @Override
  public boolean isFinished() {
    // Command finishes after shooting once
    return isAtTargetRPM && fuelSubsystem.isLaunching();
  }
}
