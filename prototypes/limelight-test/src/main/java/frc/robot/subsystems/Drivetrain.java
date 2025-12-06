package frc.robot.subsystems;

/**
 * Interface for drivetrain implementations.
 * This allows the AprilTagFollower to work with different drivetrain types.
 */
public interface Drivetrain {
    /**
     * Arcade drive method for differential drive robots.
     * @param forwardSpeed Forward/Reverse speed (positive is forward)
     * @param rotationSpeed Rotation speed (positive is clockwise)
     */
    void arcadeDrive(double forwardSpeed, double rotationSpeed);
    
    /**
     * Stop the drivetrain.
     */
    default void stop() {
        arcadeDrive(0, 0);
    }
}
