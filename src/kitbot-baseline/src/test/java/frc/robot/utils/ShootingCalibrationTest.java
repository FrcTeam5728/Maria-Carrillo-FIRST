package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import frc.robot.commands.AprilTagAimAndShootCommand.ShootingCalculator;
import frc.robot.Constants.FuelConstants;

/**
 * Unit tests for shooting calibration.
 * These tests help validate the shooting calculations for your specific robot configuration.
 */
public class ShootingCalibrationTest {
    
    private ShootingCalculator calculator;
    
    @BeforeEach
    void setUp() {
        calculator = new ShootingCalculator();
    }
    
    @Test
    @DisplayName("Test minimum distance calculation")
    void testMinDistanceCalculation() {
        double minDistance = 0.15; // 0.5 feet in meters
        double rpm = calculator.calculateRPM(minDistance);
        
        // Should be close to minimum RPM
        assertTrue(rpm >= 1000, "RPM at minimum distance should be at least 1000");
        assertTrue(rpm <= 2000, "RPM at minimum distance should not exceed 2000");
        
        System.out.println("Min distance test: " + minDistance + "m -> " + rpm + " RPM");
    }
    
    @Test
    @DisplayName("Test maximum distance calculation")
    void testMaxDistanceCalculation() {
        double maxDistance = FuelConstants.MAX_SHOOTING_DISTANCE_METERS; // 2.5 feet in meters
        double rpm = calculator.calculateRPM(maxDistance);
        
        // Should be close to maximum RPM
        assertTrue(rpm >= 4000, "RPM at maximum distance should be at least 4000");
        assertTrue(rpm <= 6000, "RPM at maximum distance should not exceed 6000");
        
        System.out.println("Max distance test: " + maxDistance + "m -> " + rpm + " RPM");
    }
    
    @Test
    @DisplayName("Test distance range validation")
    void testDistanceRangeValidation() {
        // Test distances outside valid range
        double tooCloseDistance = 0.05; // Too close
        double tooFarDistance = 10.0;   // Too far
        
        double rpmTooClose = calculator.calculateRPM(tooCloseDistance);
        double rpmTooFar = calculator.calculateRPM(tooFarDistance);
        
        // Both should be clamped to valid range
        assertEquals(calculator.calculateRPM(0.15), rpmTooClose, 0.1, "Too close distance should be clamped");
        assertEquals(calculator.calculateRPM(FuelConstants.MAX_SHOOTING_DISTANCE_METERS), rpmTooFar, 0.1, "Too far distance should be clamped");
    }
    
    @Test
    @DisplayName("Test RPM progression")
    void testRPMProgression() {
        double minDistance = 0.15;
        double maxDistance = FuelConstants.MAX_SHOOTING_DISTANCE_METERS;
        double midDistance = (minDistance + maxDistance) / 2.0;
        
        double minRPM = calculator.calculateRPM(minDistance);
        double midRPM = calculator.calculateRPM(midDistance);
        double maxRPM = calculator.calculateRPM(maxDistance);
        
        // RPM should increase with distance
        assertTrue(midRPM > minRPM, "RPM should increase from min to mid distance");
        assertTrue(maxRPM > midRPM, "RPM should increase from mid to max distance");
        
        // Mid RPM should be roughly average of min and max (for linear interpolation)
        double expectedMid = (minRPM + maxRPM) / 2.0;
        assertEquals(expectedMid, midRPM, 50, "Mid RPM should be close to average of min and max");
        
        System.out.println("RPM progression test:");
        System.out.println("  Min: " + minDistance + "m -> " + minRPM + " RPM");
        System.out.println("  Mid: " + midDistance + "m -> " + midRPM + " RPM");
        System.out.println("  Max: " + maxDistance + "m -> " + maxRPM + " RPM");
    }
    
    @Test
    @DisplayName("Test spinup time constant")
    void testSpinupTimeConstant() {
        // Verify spinup time is set correctly
        assertEquals(5.0, FuelConstants.SHOOTER_SPINUP_TIME_SECONDS, 0.1, "Spinup time should be 5 seconds");
        
        // Verify max distance is set correctly
        assertEquals(2.5, FuelConstants.MAX_SHOOTING_DISTANCE_FEET, 0.01, "Max distance should be 2.5 feet");
        assertEquals(2.5 * 0.3048, FuelConstants.MAX_SHOOTING_DISTANCE_METERS, 0.01, "Max distance in meters should be correct");
        
        // Verify calibration voltage
        assertEquals(11.5, FuelConstants.CALIBRATION_VOLTAGE, 0.1, "Calibration voltage should be 11.5V");
        
        System.out.println("Constants verification:");
        System.out.println("  Spinup time: " + FuelConstants.SHOOTER_SPINUP_TIME_SECONDS + "s");
        System.out.println("  Max distance: " + FuelConstants.MAX_SHOOTING_DISTANCE_FEET + "ft");
        System.out.println("  Calibration voltage: " + FuelConstants.CALIBRATION_VOLTAGE + "V");
    }
    
    @Test
    @DisplayName("Test wheel speed conversion")
    void testWheelSpeedConversion() {
        double testRPM = 3000.0;
        double wheelSpeed = ShootingCalculator.rpmToMetersPerSecond(testRPM);
        double convertedBack = ShootingCalculator.metersPerSecondToRPM(wheelSpeed);
        
        // Conversion should be reversible (within tolerance)
        assertEquals(testRPM, convertedBack, 1.0, "RPM conversion should be reversible");
        
        // Wheel speed should be reasonable (not too fast or slow)
        assertTrue(wheelSpeed > 5.0, "Wheel speed should be > 5 m/s at 3000 RPM");
        assertTrue(wheelSpeed < 50.0, "Wheel speed should be < 50 m/s at 3000 RPM");
        
        System.out.println("Wheel speed conversion test:");
        System.out.println("  " + testRPM + " RPM -> " + String.format("%.2f", wheelSpeed) + " m/s");
    }
    
    @Test
    @DisplayName("Test time of flight calculation")
    void testTimeOfFlight() {
        double testDistance = 1.0; // 1 meter
        double timeOfFlight = ShootingCalculator.calculateTimeOfFlight(testDistance);
        
        // Time of flight should be reasonable for 1 meter at 60 degrees
        assertTrue(timeOfFlight > 0.1, "Time of flight should be > 0.1s for 1m");
        assertTrue(timeOfFlight < 2.0, "Time of flight should be < 2.0s for 1m");
        
        System.out.println("Time of flight test:");
        System.out.println("  " + testDistance + "m -> " + String.format("%.3f", timeOfFlight) + "s");
    }
    
    @Test
    @DisplayName("Calibration validation test")
    void calibrationValidationTest() {
        System.out.println("\n=== CALIBRATION VALIDATION ===");
        
        // Test key calibration points
        double[] distancesFt = {0.5, 1.0, 1.5, 2.0, 2.5}; // feet
        double[] expectedRPMs = {1200, 2000, 3000, 4000, 5000}; // rough expectations
        
        for (int i = 0; i < distancesFt.length; i++) {
            double distanceM = distancesFt[i] * 0.3048;
            double actualRPM = calculator.calculateRPM(distanceM);
            
            System.out.println(String.format("Distance: %.1fft -> RPM: %.0f (expected ~%.0f)", 
                distancesFt[i], actualRPM, expectedRPMs[i]));
            
            // Verify RPM is in reasonable range
            assertTrue(actualRPM > 500, "RPM should be > 500 at " + distancesFt[i] + "ft");
            assertTrue(actualRPM < 7000, "RPM should be < 7000 at " + distancesFt[i] + "ft");
        }
        
        System.out.println("=============================\n");
    }
}
