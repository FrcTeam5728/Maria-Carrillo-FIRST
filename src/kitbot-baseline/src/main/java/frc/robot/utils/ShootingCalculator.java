package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

/**
 * Utility class for calculating shooting parameters for a hooded shooter.
 * Assumes a wheel diameter of 7.5 inches and a fixed launch angle of 60 degrees.
 * Includes air resistance and Magnus effect calculations.
 */
public class ShootingCalculator {
    // Physical constants
    public static final double WHEEL_DIAMETER_INCHES = 7.5;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_CIRCUMFERENCE_INCHES * 0.0254; // Convert to meters
    public static final double LAUNCH_ANGLE_DEGREES = 60.0;
    public static final double LAUNCH_ANGLE_RADIANS = Math.toRadians(LAUNCH_ANGLE_DEGREES);
    
    // Physics constants
    public static final double GRAVITY = 9.81; // m/s²
    public static final double AIR_DENSITY = 1.225; // kg/m³ at sea level, 15°C
    public static final double DRAG_COEFFICIENT = 0.25; // Reduced for dimpled ball (golf ball ~0.25)
    public static final double BALL_DIAMETER_METERS = 0.1524; // 6 inches in meters
    public static final double BALL_RADIUS = BALL_DIAMETER_METERS / 2.0;
    public static final double BALL_MASS = 0.5; // kg
    public static final double BALL_CROSS_SECTION = Math.PI * BALL_RADIUS * BALL_RADIUS;
    public static final double MAGNUS_COEFFICIENT = 1.0; // Approximate for a spinning ball
    
    // Simulation parameters
    private static final double TIME_STEP = 0.01; // seconds
    private static final double MAX_SIMULATION_TIME = 10.0; // seconds
    private static final double SPIN_RATIO = 0.5; // Ratio of surface speed to forward speed for spin
    
    /**
     * Calculates the required wheel RPM to hit a target at the given distance,
     * accounting for air resistance and Magnus effect.
     * 
     * @param targetDistanceMeters Distance to target in meters
     * @return Required wheel speed in RPM
     */
    public static double calculateRequiredRPM(double targetDistanceMeters) {
        // Start with a simple estimate
        double initialGuess = calculateInitialVelocity(targetDistanceMeters);
        
        // Binary search for the correct initial velocity
        double lowVel = 0.1;
        double highVel = 50.0; // m/s (adjust based on your system's capabilities)
        double bestVel = initialGuess;
        double minError = Double.MAX_VALUE;
        
        // Binary search for best velocity
        for (int i = 0; i < 20; i++) {
            double midVel = (lowVel + highVel) / 2.0;
            double[] result = simulateTrajectory(midVel);
            double distance = result[0];
            double error = Math.abs(distance - targetDistanceMeters);
            
            if (error < minError) {
                minError = error;
                bestVel = midVel;
            }
            
            if (distance < targetDistanceMeters) {
                lowVel = midVel;
            } else {
                highVel = midVel;
            }
            
            if (error < 0.1) { // 10cm accuracy
                break;
            }
        }
        
        // Convert velocity to wheel RPM
        // v = ωr → ω = v/r
        // RPM = (v * 60) / (2πr)
        double wheelRPM = (bestVel * 60.0) / (Math.PI * WHEEL_DIAMETER_INCHES * 0.0254);
        
        return wheelRPM;
    }
    
    /**
     * Simulates the ball's trajectory with air resistance and Magnus effect.
     * 
     * @param initialVelocity Initial velocity in m/s
     * @return Array containing [final x position, final y position, time of flight]
     */
    private static double[] simulateTrajectory(double initialVelocity) {
        // Initial conditions
        double x = 0.0;
        double y = 0.0; // Assuming ground level launch
        double vx = initialVelocity * Math.cos(LAUNCH_ANGLE_RADIANS);
        double vy = initialVelocity * Math.sin(LAUNCH_ANGLE_RADIANS);
        
        // Estimate spin based on initial velocity (simplified)
        double spinRate = (initialVelocity * SPIN_RATIO) / BALL_RADIUS; // rad/s
        
        double time = 0.0;
        
        // Run simulation until ball hits the ground or times out
        while (y >= 0 && time < MAX_SIMULATION_TIME) {
            // Calculate speed
            double speed = Math.sqrt(vx * vx + vy * vy);
            
            // Air resistance (drag) force
            double dragForce = 0.5 * AIR_DENSITY * speed * speed * DRAG_COEFFICIENT * BALL_CROSS_SECTION;
            
            // Magnus force (simplified model)
            double magnusForce = 0.5 * AIR_DENSITY * Math.PI * BALL_RADIUS * BALL_RADIUS * 
                               MAGNUS_COEFFICIENT * spinRate * speed;
            
            // Calculate acceleration components
            double ax = (-dragForce * vx / speed - magnusForce * vy / speed) / BALL_MASS;
            double ay = -GRAVITY + (-dragForce * vy / speed + magnusForce * vx / speed) / BALL_MASS;
            
            // Update velocity (Euler integration)
            vx += ax * TIME_STEP;
            vy += ay * TIME_STEP;
            
            // Update position
            x += vx * TIME_STEP;
            y += vy * TIME_STEP;
            
            time += TIME_STEP;
            
            // Early termination if ball is moving away and below launch height
            if (y < 0 && vy < 0) {
                break;
            }
        }
        
        return new double[]{x, y, time};
    }
    
    /**
     * Calculates the initial velocity needed to reach a certain distance.
     * This is a simplified calculation that ignores air resistance and is used as a starting point.
     * 
     * @param distanceMeters Horizontal distance to target in meters
     * @return Initial velocity estimate in m/s
     */
    private static double calculateInitialVelocity(double distanceMeters) {
        // Range equation: R = (v₀² * sin(2θ)) / g
        // Solving for v₀: v₀ = sqrt((R * g) / sin(2θ))
        double numerator = distanceMeters * GRAVITY;
        double denominator = Math.sin(2 * LAUNCH_ANGLE_RADIANS);
        
        // Prevent division by zero (shouldn't happen with 60° launch angle)
        if (Math.abs(denominator) < 1e-6) {
            return 0.0;
        }
        
        // Add 20% to account for air resistance (this is just a rough estimate)
        return Math.sqrt(numerator / denominator) * 1.2;
    }
    
    /**
     * Calculates the maximum theoretical range of the shooter.
     * 
     * @return Maximum range in meters
     */
    public static double calculateMaxRange() {
        // Max range occurs at 45° in a vacuum, but we'll use our fixed angle
        // R = (v₀² * sin(2θ)) / g
        // For maximum range with our fixed angle, we'll assume a reasonable max velocity
        double maxVelocity = 20.0; // m/s (adjust based on your shooter's capabilities)
        return (maxVelocity * maxVelocity * Math.sin(2 * LAUNCH_ANGLE_RADIANS)) / GRAVITY;
    }
    
    /**
     * Calculates the time of flight for a shot.
     * 
     * @param distanceMeters Distance to target in meters
     * @return Time of flight in seconds
     */
    public static double calculateTimeOfFlight(double distanceMeters) {
        double v0 = calculateInitialVelocity(distanceMeters);
        // Time of flight = (2v₀sinθ)/g
        return (2 * v0 * Math.sin(LAUNCH_ANGLE_RADIANS)) / GRAVITY;
    }
    
    /**
     * Converts RPM to meters per second at the wheel surface.
     * 
     * @param rpm Wheel speed in RPM
     * @return Surface speed in m/s
     */
    public static double rpmToMetersPerSecond(double rpm) {
        // Convert RPM to radians per second: (rpm * 2π) / 60
        // Then multiply by wheel radius: * (diameter / 2)
        return (rpm * Math.PI * WHEEL_DIAMETER_INCHES * 0.0254) / 60.0;
    }
    
    /**
     * Converts meters per second to RPM at the wheel surface.
     * 
     * @param speedMetersPerSecond Surface speed in m/s
     * @return Wheel speed in RPM
     */
    public static double metersPerSecondToRPM(double speedMetersPerSecond) {
        // Convert m/s to inches/s, then to rotations per second, then to RPM
        return (speedMetersPerSecond * 39.37 / WHEEL_CIRCUMFERENCE_INCHES) * 60.0;
    }
}
