package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FuelConstants;
import frc.robot.commands.AprilTagAimAndShootCommand.ShootingCalculator;

/**
 * Subsystem for controlling shooting distance via Xbox D-pad.
 * Allows manual adjustment of shooting distance with visual feedback.
 */
public class ShootingDistanceControl extends SubsystemBase {
  
  // Current shooting distance in feet
  private double currentDistanceFeet = 1.5; // Default to middle of range
  
  // Distance control parameters
  private static final double MIN_DISTANCE_FEET = 0.5;
  private static final double MAX_DISTANCE_FEET = FuelConstants.MAX_SHOOTING_DISTANCE_FEET;
  private static final double DISTANCE_STEP_FEET = 0.25; // 0.25 feet per D-pad press
  
  // Shooting calculator for RPM calculations
  private final ShootingCalculator shootingCalculator;
  
  public ShootingDistanceControl() {
    this.shootingCalculator = new ShootingCalculator();
  }
  
  @Override
  public void periodic() {
    // Update SmartDashboard with current values
    SmartDashboard.putNumber("Shooting Distance (ft)", currentDistanceFeet);
    SmartDashboard.putNumber("Shooting Distance (m)", currentDistanceFeet * 0.3048);
    
    // Calculate and display current RPM for this distance
    double currentRPM = shootingCalculator.calculateRPM(currentDistanceFeet * 0.3048);
    SmartDashboard.putNumber("Target RPM", currentRPM);
    
    // Display range indicators
    SmartDashboard.putString("Distance Range", 
        String.format("%.1f - %.1f ft", MIN_DISTANCE_FEET, MAX_DISTANCE_FEET));
  }
  
  /**
   * Increases shooting distance by one step.
   * Called when D-pad up is pressed.
   */
  public void increaseDistance() {
    currentDistanceFeet = Math.min(MAX_DISTANCE_FEET, currentDistanceFeet + DISTANCE_STEP_FEET);
    System.out.println("Shooting distance increased to: " + String.format("%.2f", currentDistanceFeet) + " ft");
  }
  
  /**
   * Decreases shooting distance by one step.
   * Called when D-pad down is pressed.
   */
  public void decreaseDistance() {
    currentDistanceFeet = Math.max(MIN_DISTANCE_FEET, currentDistanceFeet - DISTANCE_STEP_FEET);
    System.out.println("Shooting distance decreased to: " + String.format("%.2f", currentDistanceFeet) + " ft");
  }
  
  /**
   * Gets the current shooting distance in meters.
   * 
   * @return Current distance in meters
   */
  public double getCurrentDistanceMeters() {
    return currentDistanceFeet * 0.3048;
  }
  
  /**
   * Gets the current shooting distance in feet.
   * 
   * @return Current distance in feet
   */
  public double getCurrentDistanceFeet() {
    return currentDistanceFeet;
  }
  
  /**
   * Sets the shooting distance to a specific value.
   * 
   * @param distanceFeet Distance in feet (will be clamped to valid range)
   */
  public void setDistance(double distanceFeet) {
    this.currentDistanceFeet = Math.max(MIN_DISTANCE_FEET, Math.min(MAX_DISTANCE_FEET, distanceFeet));
    System.out.println("Shooting distance set to: " + String.format("%.2f", this.currentDistanceFeet) + " ft");
  }
  
  /**
   * Gets the target RPM for the current distance.
   * 
   * @return Target RPM for current distance
   */
  public double getTargetRPM() {
    return shootingCalculator.calculateRPM(getCurrentDistanceMeters());
  }
  
  /**
   * Resets distance to default value.
   */
  public void resetDistance() {
    currentDistanceFeet = 1.5;
    System.out.println("Shooting distance reset to: " + currentDistanceFeet + " ft");
  }
  
  /**
   * Checks if the current distance is at the minimum.
   * 
   * @return True if at minimum distance
   */
  public boolean isAtMinDistance() {
    return currentDistanceFeet <= MIN_DISTANCE_FEET;
  }
  
  /**
   * Checks if the current distance is at the maximum.
   * 
   * @return True if at maximum distance
   */
  public boolean isAtMaxDistance() {
    return currentDistanceFeet >= MAX_DISTANCE_FEET;
  }
  
  /**
   * Gets the distance as a percentage of the total range.
   * 
   * @return Distance as percentage (0-100)
   */
  public double getDistancePercentage() {
    double range = MAX_DISTANCE_FEET - MIN_DISTANCE_FEET;
    double offset = currentDistanceFeet - MIN_DISTANCE_FEET;
    return (offset / range) * 100.0;
  }
}
