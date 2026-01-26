package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Command for interacting with AprilTags. Can point to and/or approach a specific tag or the nearest visible tag.
 */
public class AprilTagCommand extends Command {
    private final SwerveSubsystem swerve;
    private final LimelightSubsystem limelight;
    private final Odometry odometry;
    private final Integer targetTagId; // Null means use nearest tag
    private final DoubleSupplier targetDistanceMeters; // Negative means don't approach, just point
    
    // PID controllers
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    // PID constants
    private static final double LINEAR_P = 1.0;
    private static final double LINEAR_I = 0.0;
    private static final double LINEAR_D = 0.1;
    private static final double ROTATION_P = 0.5;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.1;
    
    // Tolerances
    private static final double POSITION_TOLERANCE_METERS = 0.05; // 5cm
    private static final double ANGLE_TOLERANCE_RADIANS = Math.toRadians(2.0);
    
    // State
    private boolean isPositionControl;

    /**
     * Creates a new command to point to and approach a specific AprilTag.
     * 
     * @param swerve The swerve drive subsystem
     * @param limelight The Limelight subsystem for vision processing
     * @param odometry The odometry subsystem for pose estimation
     * @param targetTagId The ID of the tag to target
     * @param targetDistanceMeters Distance to maintain from the tag (negative for pointing only)
     */
    public AprilTagCommand(SwerveSubsystem swerve, LimelightSubsystem limelight, Odometry odometry, int targetTagId, double targetDistanceMeters) {
        this(swerve, limelight, odometry, targetTagId, () -> targetDistanceMeters);
    }
    
    /**
     * Creates a new command to point to and approach a specific AprilTag with dynamic target distance.
     * 
     * @param swerve The swerve drive subsystem
     * @param limelight The Limelight subsystem for vision processing
     * @param odometry The odometry subsystem for pose estimation
     * @param targetTagId The ID of the tag to target
     * @param targetDistanceMeters Supplier for the distance to maintain from the tag (negative for pointing only)
     */
    public AprilTagCommand(SwerveSubsystem swerve, LimelightSubsystem limelight, Odometry odometry, int targetTagId, DoubleSupplier targetDistanceMeters) {
        this(swerve, limelight, odometry, Integer.valueOf(targetTagId), targetDistanceMeters);
    }
    
    /**
     * Creates a new command to point to and approach the nearest AprilTag.
     * 
     * @param swerve The swerve drive subsystem
     * @param limelight The Limelight subsystem for vision processing
     * @param odometry The odometry subsystem for pose estimation
     * @param targetDistanceMeters Supplier for the distance to maintain from the tag (negative for pointing only)
     */
    public AprilTagCommand(SwerveSubsystem swerve, LimelightSubsystem limelight, Odometry odometry, DoubleSupplier targetDistanceMeters) {
        this(swerve, limelight, odometry, null, targetDistanceMeters);
    }
    
    private AprilTagCommand(SwerveSubsystem swerve, LimelightSubsystem limelight, Odometry odometry, Integer targetTagId, DoubleSupplier targetDistanceMeters) {
        this.swerve = swerve;
        this.limelight = limelight;
        this.odometry = odometry;
        this.targetTagId = targetTagId;
        this.targetDistanceMeters = targetDistanceMeters;
        
        // Initialize PID controllers
        xController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
        yController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);
        rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        
        // Configure controllers
        xController.setTolerance(POSITION_TOLERANCE_METERS);
        yController.setTolerance(POSITION_TOLERANCE_METERS);
        rotationController.setTolerance(ANGLE_TOLERANCE_RADIANS);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Add requirements
        addRequirements(swerve, limelight, odometry);
    }

    @Override
    public void initialize() {
        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();
        
        // Turn on LED for vision processing
        limelight.setLedMode(LimelightSubsystem.LedMode.ON);
        
        // Determine control mode
        isPositionControl = targetDistanceMeters.getAsDouble() >= 0;
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget()) {
            // No target in sight, stop the robot
            swerve.drive(new Translation2d(0, 0), 0, true);
            return;
        }
        
        // Get current robot pose from odometry
        Pose2d robotPose = odometry.getPose();
        
        // Get target information from limelight
        double tx = limelight.getTx();
        double ty = limelight.getTy();
        double ta = limelight.getTa();
        double tid = limelight.getTid();
        
        // If we're looking for a specific tag and this isn't it, skip
        if (targetTagId != null && tid != targetTagId) {
            swerve.drive(new Translation2d(0, 0), 0, true);
            return;
        }
        
        // Convert angles to radians for calculations
        double txRad = Math.toRadians(tx);
        double tyRad = Math.toRadians(ty);
        
        // Calculate distance to target using camera parameters and known tag size
        // This is a simplified model - you may need to adjust based on your setup
        double distanceToTarget = 0.0; // Calculate based on your camera parameters
        
        // Calculate desired position relative to robot
        double desiredX = distanceToTarget * Math.cos(txRad);
        double desiredY = distanceToTarget * Math.sin(txRad);
        
        // If we have a target distance, adjust the desired position
        double targetDistance = targetDistanceMeters.getAsDouble();
        if (targetDistance >= 0) {
            // Calculate the vector from current position to target
            double currentDistance = Math.hypot(desiredX, desiredY);
            double scale = (currentDistance - targetDistance) / currentDistance;
            
            // Scale the vector to maintain the target distance
            desiredX *= scale;
            desiredY *= scale;
        }
        
        // Calculate PID outputs
        double xSpeed = xController.calculate(0, desiredX);
        double ySpeed = yController.calculate(0, desiredY);
        double rotationSpeed = rotationController.calculate(0, txRad);
        
        // Drive the robot
        swerve.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true);
        
        // Log data
        SmartDashboard.putNumber("AprilTag/TX", tx);
        SmartDashboard.putNumber("AprilTag/TY", ty);
        SmartDashboard.putNumber("AprilTag/TA", ta);
        SmartDashboard.putNumber("AprilTag/Distance", distanceToTarget);
    }

    @Override
    public boolean isFinished() {
        // Command ends when both position and rotation are at setpoints (if they're being controlled)
        boolean positionDone = !isPositionControl || (xController.atSetpoint() && yController.atSetpoint());
        boolean rotationDone = rotationController.atSetpoint();
        return positionDone && rotationDone;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        swerve.drive(new Translation2d(0, 0), 0, true);
        
        // Turn off LED when done
        limelight.setLedMode(LimelightSubsystem.LedMode.OFF);
    }
}
