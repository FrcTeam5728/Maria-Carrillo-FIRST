package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
// TODO: Replace with NavX2 when dependencies are installed
// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.SerialPort;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract base class for drive subsystems.
 * Provides a common interface and shared implementation for different drivetrain motor controller implementations.
 * 
 * This class manages the DifferentialDrive instance and provides a default implementation
 * of driveArcade() that subclasses can override if needed.
 */
public abstract class DriveSubsystem extends SubsystemBase {
    /**
     * The differential drive instance shared by all drive subsystems.
     * Protected so subclasses can initialize it in their constructors.
     */
    protected DifferentialDrive drive;
    
    // Odometry for tracking robot position
    protected DifferentialDriveOdometry odometry;
    protected Pose2d pose = new Pose2d();
    
    // Encoders for wheel position tracking
    protected Encoder leftEncoder;
    protected Encoder rightEncoder;
    
    // Wheel positions for odometry
    protected DifferentialDriveWheelPositions wheelPositions;
    
    // Physical constants
    protected static final double TRACK_WIDTH_METERS = 0.7; // KitBot track width
    protected static final double WHEEL_RADIUS_METERS = 0.0762; // 3 inches
    
    // Simple position tracking for simulation
    protected double simulatedX = 0.0;
    protected double simulatedY = 0.0;
    protected double simulatedHeading = 0.0;
    
    // Debug counters
    protected int debugCounter = 0;
    protected int simCounter = 0;
    protected int errorCounter = 0;
    
    // Manual simulation control for testing
    protected double manualLeftSpeed = 0.0;
    protected double manualRightSpeed = 0.0;
    
    // Movement inversion state
    private boolean movementInverted = false;
    
    /**
     * Update odometry - should be called periodically
     */
    @Override
    public void periodic() {
        // Read manual simulation speeds from SmartDashboard
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            manualLeftSpeed = SmartDashboard.getNumber("Simulation/LeftSpeed", 0.0);
            manualRightSpeed = SmartDashboard.getNumber("Simulation/RightSpeed", 0.0);
            
            // Check for reset command
            if (SmartDashboard.getBoolean("Simulation/Reset", false)) {
                resetSimulation();
                SmartDashboard.putBoolean("Simulation/Reset", false); // Clear reset flag
            }
        }
        
        // Update odometry with encoder readings only (no gyro)
        if (leftEncoder != null && rightEncoder != null && odometry != null) {
            // Get current wheel positions
            double leftDistance = leftEncoder.getDistance();
            double rightDistance = rightEncoder.getDistance();
            wheelPositions = new DifferentialDriveWheelPositions(leftDistance, rightDistance);
            
            // Update odometry with current heading (no gyro angle available)
            pose = odometry.update(pose.getRotation(), wheelPositions);
            
            // Debug: Print odometry updates (every 1 second to avoid spam)
            debugCounter++;
            if (debugCounter % 50 == 0) {
                System.out.println("Odometry: X=" + String.format("%.3f", pose.getX()) + 
                                 ", Y=" + String.format("%.3f", pose.getY()) + 
                                 ", Heading=" + String.format("%.1f", pose.getRotation().getDegrees()));
            }
        } else if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            // Simple simulation update based on motor commands
            updateSimulationPose();
            pose = new Pose2d(simulatedX, simulatedY, new Rotation2d(simulatedHeading));
            
            // Debug: Print simulation updates
            simCounter++;
            if (simCounter % 50 == 0) {
                System.out.println("Simulation: X=" + String.format("%.3f", simulatedX) + 
                                 ", Y=" + String.format("%.3f", simulatedY) + 
                                 ", Heading=" + String.format("%.1f", Math.toDegrees(simulatedHeading)) +
                                 ", Speeds(L,R)=" + String.format("%.2f,%.2f", manualLeftSpeed, manualRightSpeed));
            }
        } else {
            // Debug: Print missing sensors
            errorCounter++;
            if (errorCounter % 100 == 0) {
                System.out.println("Odometry not updating - Missing sensors:");
                System.out.println("  Left encoder: " + (leftEncoder != null ? "OK" : "NULL"));
                System.out.println("  Right encoder: " + (rightEncoder != null ? "OK" : "NULL"));
                System.out.println("  Odometry: " + (odometry != null ? "OK" : "NULL"));
            }
        }
        
        // Publish pose and sensor data to SmartDashboard for debugging
        SmartDashboard.putNumber("Drive/Odometry/X", pose.getX());
        SmartDashboard.putNumber("Drive/Odometry/Y", pose.getY());
        SmartDashboard.putNumber("Drive/Odometry/Heading", pose.getRotation().getDegrees());
        
        if (leftEncoder != null && rightEncoder != null) {
            SmartDashboard.putNumber("Drive/Encoder/Left", leftEncoder.getDistance());
            SmartDashboard.putNumber("Drive/Encoder/Right", rightEncoder.getDistance());
        }
    }
    
    /**
     * Simple simulation update
     */
    private void updateSimulationPose() {
        if (drive != null) {
            // Get current motor speeds from differential drive
            // Note: This is a simplified approach - in a real simulation you'd
            // use actual motor controller objects or simulation framework
            
            // Use manual speeds for testing (set via SmartDashboard)
            double leftSpeed = manualLeftSpeed;
            double rightSpeed = manualRightSpeed;
            
            // Update position based on differential drive kinematics
            double linearVelocity = (leftSpeed + rightSpeed) / 2.0;
            double angularVelocity = (rightSpeed - leftSpeed) / TRACK_WIDTH_METERS;
            
            // Update heading
            simulatedHeading += angularVelocity * 0.02; // 20ms period
            
            // Update position
            simulatedX += linearVelocity * Math.cos(simulatedHeading) * 0.02;
            simulatedY += linearVelocity * Math.sin(simulatedHeading) * 0.02;
        }
    }
    
    /**
     * Get current robot pose
     * @return Current robot pose
     */
    public Pose2d getPose() {
        return pose;
    }
    
    /**
     * Get current heading as Rotation2d
     * @return Current heading from pose
     */
    public Rotation2d getHeading() {
        return pose.getRotation();
    }
    
    /**
     * Set manual simulation speeds for testing
     * @param leftSpeed Left motor speed (-1.0 to 1.0)
     * @param rightSpeed Right motor speed (-1.0 to 1.0)
     */
    public void setSimulationSpeeds(double leftSpeed, double rightSpeed) {
        manualLeftSpeed = leftSpeed;
        manualRightSpeed = rightSpeed;
        
        // Debug output
        System.out.println("Simulation speeds set: Left=" + leftSpeed + ", Right=" + rightSpeed);
    }
    
    /**
     * Reset simulation position
     */
    public void resetSimulation() {
        simulatedX = 0.0;
        simulatedY = 0.0;
        simulatedHeading = 0.0;
        manualLeftSpeed = 0.0;
        manualRightSpeed = 0.0;
        System.out.println("Simulation position reset");
    }
    
    /**
     * Reset odometry to a specific pose
     * @param pose The pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        this.pose = pose;
        simulatedX = pose.getX();
        simulatedY = pose.getY();
        simulatedHeading = pose.getRotation().getRadians();
        
        // Reset odometry with current pose
        if (odometry != null && wheelPositions != null) {
            odometry.resetPosition(pose.getRotation(), wheelPositions, pose);
        }
        
        // Reset encoders
        if (leftEncoder != null && rightEncoder != null) {
            leftEncoder.reset();
            rightEncoder.reset();
            wheelPositions = new DifferentialDriveWheelPositions(0.0, 0.0);
        }
    }
    
    /**
     * Initialize odometry system
     */
    protected void initializeOdometry() {
        odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
        wheelPositions = new DifferentialDriveWheelPositions(0.0, 0.0);
        pose = new Pose2d();
        System.out.println("Drive odometry initialized");
    }
    
    /**
     * Initialize encoders for odometry
     */
    protected void initializeEncoders(int leftEncoderId, int rightEncoderId) {
        leftEncoder = new Encoder(leftEncoderId, leftEncoderId + 1);
        rightEncoder = new Encoder(rightEncoderId, rightEncoderId + 1);
        
        // Configure encoders
        leftEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS_METERS / 360.0);
        rightEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS_METERS / 360.0);
        rightEncoder.setReverseDirection(true);
        
        leftEncoder.reset();
        rightEncoder.reset();
        
        System.out.println("Drive encoders initialized");
    }

    /**
     * Creates an arcade drive command.
     * Provides a default implementation using the drive field.
     * Subclasses should initialize the drive field in their constructor before this is called.
     * 
     * @param forward DoubleSupplier for forward/backward speed
     * @param rotation DoubleSupplier for rotation speed
     * @return Command to drive the robot
     */
    public Command driveArcade(DoubleSupplier forward, DoubleSupplier rotation) {
        return this.run(() -> {
            double forwardSpeed = forward.getAsDouble();
            double rotationSpeed = rotation.getAsDouble();
            
            // Apply movement inversion to translation only (forward/backward)
            if (movementInverted) {
                forwardSpeed = -forwardSpeed; // Only invert forward/backward
                // Rotation stays normal
            }
            
            drive.arcadeDrive(forwardSpeed, rotationSpeed);
        });
    }
    
    /**
     * Toggles movement inversion state.
     * When inverted, all forward/backward and left/right movement is reversed.
     */
    public void toggleMovementInversion() {
        movementInverted = !movementInverted;
        System.out.println("Movement inversion " + (movementInverted ? "ENABLED" : "DISABLED"));
        SmartDashboard.putBoolean("Drive/MovementInverted", movementInverted);
    }
    
    /**
     * Gets the current movement inversion state.
     * 
     * @return True if movement is inverted
     */
    public boolean isMovementInverted() {
        return movementInverted;
    }
    
    /**
     * Sets the movement inversion state.
     * 
     * @param inverted Whether movement should be inverted
     */
    public void setMovementInverted(boolean inverted) {
        this.movementInverted = inverted;
        System.out.println("Movement inversion " + (movementInverted ? "ENABLED" : "DISABLED"));
        SmartDashboard.putBoolean("Drive/MovementInverted", movementInverted);
    }
}
