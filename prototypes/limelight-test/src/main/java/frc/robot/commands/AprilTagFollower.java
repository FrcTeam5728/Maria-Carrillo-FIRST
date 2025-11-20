package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * A command that follows the nearest AprilTag using a Limelight camera.
 * This command will automatically rotate and move toward the target AprilTag
 * while maintaining a safe distance.
 */
public class AprilTagFollower extends Command {
    private final LimelightSubsystem limelight;
    private final Drivetrain drivetrain;
    private final int targetId;  // -1 means any target
    
    // PID Controllers for rotation and distance
    private final PIDController rotationController;
    private final PIDController distanceController;
    
    // Configuration constants (tune these for your robot)
    private static final double MAX_ROTATION_SPEED = 0.5;  // Max rotation speed (0 to 1)
    private static final double MAX_FORWARD_SPEED = 0.5;   // Max forward speed (0 to 1)
    private static final double MIN_TARGET_AREA = 0.5;     // Minimum target area to consider valid
    private static final double TARGET_DISTANCE = 1.0;     // Desired distance from target in meters
    private static final double ANGLE_TOLERANCE = 2.0;     // Degrees of error allowed
    private static final double DISTANCE_TOLERANCE = 0.1;  // Meters of error allowed
    
    /**
     * Creates a new AprilTagFollower command.
     * 
     * @param limelight The Limelight subsystem
     * @param drivetrain The drivetrain to control
     * @param targetId The ID of the AprilTag to follow (-1 for any tag)
     */
    public AprilTagFollower(LimelightSubsystem limelight, Drivetrain drivetrain, int targetId) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.targetId = targetId;
        
        // Initialize PID controllers with tuned values
        rotationController = new PIDController(0.03, 0.0, 0.003);
        rotationController.setTolerance(ANGLE_TOLERANCE);
        rotationController.enableContinuousInput(-180, 180);
        
        distanceController = new PIDController(0.5, 0.1, 0.05);
        distanceController.setTolerance(DISTANCE_TOLERANCE);
        distanceController.setSetpoint(TARGET_DISTANCE);
        
        // Add to SmartDashboard for tuning
        SmartDashboard.putData("AprilTagFollower/Rotation PID", rotationController);
        SmartDashboard.putData("AprilTagFollower/Distance PID", distanceController);
        
        // This command requires the drivetrain and limelight
        if (drivetrain instanceof SubsystemBase) {
            addRequirements((SubsystemBase) drivetrain);
        }
        if (limelight != null) {
            addRequirements(limelight);
        }
    }
    
    @Override
    public void initialize() {
        // Reset controllers and enable the limelight
        rotationController.reset();
        distanceController.reset();
        limelight.setLedMode(LimelightSubsystem.LedMode.ON);
        limelight.setCamMode(LimelightSubsystem.CamMode.VISION);
    }
    
    @Override
    public void execute() {
        updateSmartDashboard();
        
        if (!hasValidTarget()) {
            // No valid target, stop the robot
            drivetrain.stop();
            return;
        }
        
        // Calculate rotation and forward speeds using PID
        double tx = limelight.getTx();
        double rotationSpeed = -rotationController.calculate(tx);
        
        double distance = limelight.getDistanceToTarget();
        double forwardSpeed = 0;
        
        // Only move forward/backward if we have a valid distance
        if (distance > 0) {
            forwardSpeed = -distanceController.calculate(distance);
        }
        
        // Apply speed limits
        rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationSpeed));
        forwardSpeed = Math.max(-MAX_FORWARD_SPEED, Math.min(MAX_FORWARD_SPEED, forwardSpeed));
        
        // Apply deadband
        if (Math.abs(rotationSpeed) < 0.05) rotationSpeed = 0;
        if (Math.abs(forwardSpeed) < 0.05) forwardSpeed = 0;
        
        // Drive the robot
        drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Turn off the limelight LEDs and stop the drivetrain when command ends
        limelight.setLedMode(LimelightSubsystem.LedMode.OFF);
        drivetrain.stop();
    }
    
    @Override
    public boolean isFinished() {
        // End the command when we're aligned with the target
        return isAligned();
    }
    
    /**
     * Checks if the robot is aligned with the target AprilTag.
     * @return True if aligned, false otherwise
     */
    public boolean isAligned() {
        if (!hasValidTarget()) return false;
        return Math.abs(limelight.getTx()) < ANGLE_TOLERANCE && 
               Math.abs(limelight.getDistanceToTarget() - TARGET_DISTANCE) < DISTANCE_TOLERANCE;
    }
    
    /**
     * Checks if the current target is valid for following.
     * @return True if the target is valid, false otherwise
     */
    private boolean hasValidTarget() {
        if (!limelight.hasTarget()) return false;
        if (targetId != -1 && limelight.getTargetId() != targetId) return false;
        if (limelight.getTa() < MIN_TARGET_AREA) return false;
        return true;
    }
    
    /**
     * Updates the SmartDashboard with current values.
     */
    private void updateSmartDashboard() {
        SmartDashboard.putBoolean("AprilTagFollower/Has Target", limelight.hasTarget());
        SmartDashboard.putNumber("AprilTagFollower/Target ID", limelight.getTargetId());
        SmartDashboard.putNumber("AprilTagFollower/Target Area", limelight.getTa());
        SmartDashboard.putNumber("AprilTagFollower/Distance", limelight.getDistanceToTarget());
        SmartDashboard.putBoolean("AprilTagFollower/Is Aligned", isAligned());
    }
    
    /**
     * Creates a command to follow a specific AprilTag.
     * @param limelight The Limelight subsystem
     * @param drivetrain The drivetrain to control
     * @param targetId The ID of the AprilTag to follow
     * @return A command that will follow the specified AprilTag
     */
    public static Command createFollowTagCommand(LimelightSubsystem limelight, Drivetrain drivetrain, int targetId) {
        return new AprilTagFollower(limelight, drivetrain, targetId);
    }
}
