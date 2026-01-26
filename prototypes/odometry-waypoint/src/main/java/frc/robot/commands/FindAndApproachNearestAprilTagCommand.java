package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to find and approach the nearest AprilTag using Limelight.
 * Uses PID control to drive towards the target while maintaining alignment.
 */

public class FindAndApproachNearestAprilTagCommand extends Command {
    private final SwerveSubsystem swerve;
    private final LimelightSubsystem limelight;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    // PID Constants - these may need tuning
    private static final double X_kP = 0.1, X_kI = 0.0, X_kD = 0.01;  // For strafe (x) control
    private static final double Y_kP = 0.5, Y_kI = 0.0, Y_kD = 0.05;  // For forward (y) control
    private static final double ROT_kP = 0.03, ROT_kI = 0.0, ROT_kD = 0.005; // For rotation control
    
    // Camera and target configuration (in meters and degrees)
    private static final double TARGET_HEIGHT = 0.5;      // Height of AprilTag from floor
    private static final double CAMERA_HEIGHT = 0.3;      // Height of camera from floor
    private static final double CAMERA_ANGLE = 30.0;      // Angle of camera from horizontal (degrees)
    private static final double TARGET_DISTANCE = 1.0;    // Desired distance from target (meters)
    
    // Maximum speeds
    private static final double MAX_LINEAR_SPEED = 1.5;   // m/s
    private static final double MAX_ANGULAR_SPEED = Math.PI; // rad/s
    
    public FindAndApproachNearestAprilTagCommand(SwerveSubsystem swerve, LimelightSubsystem limelight) {
        this.swerve = swerve;
        this.limelight = limelight;
        
        // Initialize PID controllers with separate gains for each axis
        xController = new PIDController(X_kP, X_kI, X_kD);
        yController = new PIDController(Y_kP, Y_kI, Y_kD);
        rotationController = new PIDController(ROT_kP, ROT_kI, ROT_kD);
        
        // Set tolerance for PID controllers
        xController.setTolerance(0.05);          // 5cm tolerance for X position
        yController.setTolerance(0.05);          // 5cm tolerance for Y position
        rotationController.setTolerance(2.0);    // 2 degree tolerance for rotation
        
        // Set setpoints (we want to center the target and reach TARGET_DISTANCE)
        xController.setSetpoint(0);              // Center X (left/right)
        yController.setSetpoint(TARGET_DISTANCE); // Target distance (forward/back)
        rotationController.setSetpoint(0);        // Center rotation
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();
        
        // Turn on Limelight LED when command starts
        limelight.setLEDMode(LimelightSubsystem.LEDMode.ON);
        
        // Set Limelight pipeline to AprilTag detection (assuming pipeline 0 is for AprilTags)
        limelight.setPipeline(0);
        
        SmartDashboard.putString("AprilTag/Status", "Searching for target...");
    }
    
    @Override
    public void execute() {
        // Check if we have a valid target
        if (limelight.hasTarget()) {
            // Get target information from Limelight
            double tx = limelight.getX();  // Horizontal offset from crosshair to target (-27 to 27 degrees)
            double ty = limelight.getY();  // Vertical offset from crosshair to target (-20.5 to 20.5 degrees)
            double ta = limelight.getArea(); // Target area (0% to 100% of image)
            
            // Calculate distance to target using trigonometry
            // distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(Math.toRadians(CAMERA_ANGLE + ty))
            double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / 
                           Math.tan(Math.toRadians(CAMERA_ANGLE + ty));
            
            // Calculate desired velocities using PID controllers
            double xSpeed = -xController.calculate(tx / 27.0); // Normalize tx to -1 to 1
            double ySpeed = yController.calculate(distance);    // Drive to maintain target distance
            double rotationSpeed = -rotationController.calculate(Math.toRadians(tx));
            
            // Scale speeds to maximum values
            xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), MAX_LINEAR_SPEED), xSpeed);
            ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), MAX_LINEAR_SPEED), ySpeed);
            rotationSpeed = Math.copySign(Math.min(Math.abs(rotationSpeed), MAX_ANGULAR_SPEED), rotationSpeed);
            
            // Drive the robot (field-relative)
            swerve.drive(
                new Translation2d(ySpeed, xSpeed), // Forward/back, then left/right
                rotationSpeed,
                true
            );
            
            // Debug information
            SmartDashboard.putNumber("AprilTag/tx", tx);
            SmartDashboard.putNumber("AprilTag/ty", ty);
            SmartDashboard.putNumber("AprilTag/Area", ta);
            SmartDashboard.putNumber("AprilTag/Distance", distance);
            SmartDashboard.putNumber("AprilTag/X Speed", xSpeed);
            SmartDashboard.putNumber("AprilTag/Y Speed", ySpeed);
            SmartDashboard.putNumber("AprilTag/Rotation", rotationSpeed);
            SmartDashboard.putString("AprilTag/Status", "Tracking target");
        } else {
            // No target found, stop the robot and rotate slowly to search
            swerve.drive(
                new Translation2d(0, 0),
                0.3, // Slow rotation to search for target
                true
            );
            SmartDashboard.putString("AprilTag/Status", "Searching for target...");
        }
    }
    
    /**
     * Checks if we're close enough to the target
     * @return true if we've reached the target position and orientation
     */
    private boolean isAtTarget() {
        if (!limelight.hasTarget()) return false;
        
        double tx = limelight.getX();
        double ty = limelight.getY();
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / 
                        Math.tan(Math.toRadians(CAMERA_ANGLE + ty));
        
        boolean atDistance = Math.abs(distance - TARGET_DISTANCE) < 0.1; // Within 10cm
        boolean aligned = Math.abs(tx) < 3.0; // Within 3 degrees
        
        return atDistance && aligned;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        swerve.drive(new Translation2d(0, 0), 0, true);
        
        // Turn off Limelight LED to save power
        limelight.setLEDMode(LimelightSubsystem.LEDMode.OFF);
        
        if (interrupted) {
            SmartDashboard.putString("AprilTag/Status", "Interrupted");
        } else {
            SmartDashboard.putString("AprilTag/Status", "Target reached");
        }
    }
    
    @Override
    public boolean isFinished() {
        // End when we're close enough to the target
        boolean atTarget = isAtTarget();
        SmartDashboard.putBoolean("AprilTag/AtTarget", atTarget);
        return atTarget;
    }
}
