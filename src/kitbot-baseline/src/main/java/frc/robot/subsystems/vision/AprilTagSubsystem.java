package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.constraints.ConstraintManager;
import frc.robot.subsystems.vision.constraints.DistanceConstraint;
import frc.robot.subsystems.vision.constraints.PositionConstraint;
import frc.robot.subsystems.vision.constraints.RotationConstraint;
import frc.robot.subsystems.vision.limelight.LimelightTarget;

import java.util.Optional;

/**
 * Subsystem for handling AprilTag detection and tracking using Limelight.
 * This subsystem provides methods to get the robot's pose relative to detected AprilTags
 * and calculate chassis speeds to align with targets based on configured constraints.
 */
public class AprilTagSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final ConstraintManager constraintManager;
    private int targetId = -1; // -1 means any visible target
    
    // Limelight NetworkTable entries
    private final NetworkTableEntry tv;  // Whether the limelight has any valid targets (0 or 1)
    private final NetworkTableEntry tx;   // Horizontal offset from crosshair to target (-27 to 27 degrees)
    private final NetworkTableEntry ty;   // Vertical offset from crosshair to target (-20.5 to 20.5 degrees)
    private final NetworkTableEntry ta;   // Target area (0% to 100% of image)
    private final NetworkTableEntry tid;  // ID of the primary in-view AprilTag
    
    // Camera constants - update these based on your Limelight model and mounting
    private static final double MOUNT_ANGLE_DEGREES = 0.0; // Angle from horizontal in degrees
    private static final double LENS_HEIGHT_METERS = 0.0;   // Height of the camera lens from the floor
    private static final double TARGET_HEIGHT_METERS = 0.0; // Height of the target from the floor

    /**
     * Creates a new AprilTagSubsystem that uses the default Limelight table (limelight).
     */
    public AprilTagSubsystem() {
        this("limelight");
    }
    
    /**
     * Creates a new AprilTagSubsystem that uses the specified Limelight table.
     * 
     * @param limelightName The name of the Limelight as configured in the web interface
     */
    public AprilTagSubsystem(String limelightName) {
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
        this.constraintManager = new ConstraintManager();
        
        // Initialize NetworkTable entries
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tid = limelightTable.getEntry("tid");
        
        // Configure Limelight pipeline and mode
        setPipeline(0); // Default pipeline for AprilTag detection
        setDriverMode(false); // Vision processing mode
        
        // Initialize with default constraints
        initializeDefaultConstraints();
        
        // Load the AprilTag field layout
        try {
            aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
    }

    /**
     * Configures the default constraints for the subsystem.
     * These can be overridden using the constraint manager.
     */
    private void initializeDefaultConstraints() {
        // Add default constraints with reasonable PID values
        constraintManager.addConstraint("distance", new DistanceConstraint(
            "Distance", 1.0, 0.5, 0.0, 0.1));
        
        constraintManager.addConstraint("rotation", new RotationConstraint(
            "Rotation", 0.1, 0.0, 0.01));
        
        constraintManager.addConstraint("position", new PositionConstraint(
            "Position", 0.0, 0.0, 0.5, 0.0, 0.1));
    }

    /**
     * Sets the target AprilTag ID to track. Set to -1 to track any visible target.
     * 
     * @param id The ID of the AprilTag to track, or -1 for any target
     */
    public void setTargetId(int id) {
        this.targetId = id;
    }
    
    /**
     * Configures the Limelight's pipeline.
     * 
     * @param pipeline The pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Sets the camera mode (driver or vision processing).
     * 
     * @param driverMode True for driver mode (increased exposure, no processing),
     *                  false for vision processing mode
     */
    public void setDriverMode(boolean driverMode) {
        limelightTable.getEntry("camMode").setNumber(driverMode ? 1 : 0);
    }

    /**
     * Checks if the Limelight has a valid target.
     * 
     * @return True if a target is detected, false otherwise
     */
    public boolean hasTarget() {
        return tv.getDouble(0) == 1 && (targetId == -1 || getTargetId() == targetId);
    }
    
    /**
     * Gets the ID of the currently detected AprilTag.
     * 
     * @return The AprilTag ID, or -1 if no target is detected
     */
    public int getTargetId() {
        return (int)tid.getDouble(-1);
    }
    
    /**
     * Gets the horizontal offset from the crosshair to the target.
     * 
     * @return Horizontal offset in degrees (-27 to 27), or 0 if no target
     */
    public double getTargetX() {
        return hasTarget() ? tx.getDouble(0) : 0;
    }
    
    /**
     * Gets the vertical offset from the crosshair to the target.
     * 
     * @return Vertical offset in degrees (-20.5 to 20.5), or 0 if no target
     */
    public double getTargetY() {
        return hasTarget() ? ty.getDouble(0) : 0;
    }
    
    /**
     * Gets the target area as a percentage of the image.
     * 
     * @return Target area (0% to 100%), or 0 if no target
     */
    public double getTargetArea() {
        return hasTarget() ? ta.getDouble(0) : 0;
    }

    /**
     * Gets the constraint manager for this subsystem.
     * 
     * @return The constraint manager instance
     */
    public ConstraintManager getConstraintManager() {
        return constraintManager;
    }

    /**
     * Gets the current LimelightTarget with all available data.
     * 
     * @return A LimelightTarget object containing the current target data
     */
    public LimelightTarget getLimelightTarget() {
        if (!hasTarget()) {
            return LimelightTarget.empty();
        }
        
        return new LimelightTarget(
            true,
            getTargetX(),
            getTargetY(),
            getTargetArea(),
            getTargetId(),
            getDistanceToTarget()
        );
    }

    /**
     * Calculates the distance to the target using the camera's vertical angle.
     * 
     * @return Distance to the target in meters, or 0 if no target
     */
    public double getDistanceToTarget() {
        if (!hasTarget()) return 0;
        
        // Calculate distance using the vertical angle and known heights
        double angleToTarget = MOUNT_ANGLE_DEGREES + getTargetY();
        double heightDifference = TARGET_HEIGHT_METERS - LENS_HEIGHT_METERS;
        
        // Distance = (height difference) / tan(angle)
        // Convert angle to radians for Math.tan()
        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }

    /**
     * Gets the forward speed needed to maintain target distance.
     * 
     * @return Forward speed in meters per second, or 0 if no target
     */
    public double getForwardSpeed() {
        return getTargetSpeeds()
            .map(speeds -> speeds.vxMetersPerSecond)
            .orElse(0.0);
    }
    
    /**
     * Gets the rotation speed needed to face the target.
     * 
     * @return Rotation speed in radians per second, or 0 if no target
     */
    public double getRotationSpeed() {
        return getTargetSpeeds()
            .map(speeds -> speeds.omegaRadiansPerSecond)
            .orElse(0.0);
    }

    /**
     * Calculates the chassis speeds needed to align with the target using the constraint system.
     * 
     * @return ChassisSpeeds with the required velocities, or empty if no target
     */
    public Optional<ChassisSpeeds> getTargetSpeeds() {
        if (!hasTarget()) return Optional.empty();

        LimelightTarget target = getLimelightTarget();
        Optional<Pose2d> currentPose = getRobotPose();
        
        // Get all active constraints and calculate their contributions
        Optional<ChassisSpeeds> distanceSpeeds = constraintManager.getConstraint("distance")
            .filter(constraint -> constraint.isActive())
            .map(constraint -> constraint.calculate(target, currentPose))
            .orElse(Optional.empty());
            
        Optional<ChassisSpeeds> rotationSpeeds = constraintManager.getConstraint("rotation")
            .filter(constraint -> constraint.isActive())
            .map(constraint -> constraint.calculate(target, currentPose))
            .orElse(Optional.empty());
            
        Optional<ChassisSpeeds> positionSpeeds = constraintManager.getConstraint("position")
            .filter(constraint -> constraint.isActive())
            .map(constraint -> constraint.calculate(target, currentPose))
            .orElse(Optional.empty());

        // Combine all constraint outputs
        double vx = 0, vy = 0, omega = 0;
        
        if (distanceSpeeds.isPresent()) {
            ChassisSpeeds speeds = distanceSpeeds.get();
            vx += speeds.vxMetersPerSecond;
            vy += speeds.vyMetersPerSecond;
            omega += speeds.omegaRadiansPerSecond;
        }
        
        if (rotationSpeeds.isPresent()) {
            ChassisSpeeds speeds = rotationSpeeds.get();
            vx += speeds.vxMetersPerSecond;
            vy += speeds.vyMetersPerSecond;
            omega += speeds.omegaRadiansPerSecond;
        }
        
        if (positionSpeeds.isPresent()) {
            ChassisSpeeds speeds = positionSpeeds.get();
            vx += speeds.vxMetersPerSecond;
            vy += speeds.vyMetersPerSecond;
            omega += speeds.omegaRadiansPerSecond;
        }
        
        return Optional.of(new ChassisSpeeds(vx, vy, omega));
    }

    /**
     * Gets the robot's estimated field-relative pose based on AprilTag detection.
     * 
     * @return The estimated robot pose, or empty if no valid target is detected
     */
    public Optional<Pose2d> getRobotPose() {
        if (!hasTarget()) return Optional.empty();
        
        try {
            int tagId = getTargetId();
            var targetPose = aprilTagFieldLayout.getTagPose(tagId);
            
            if (targetPose.isEmpty()) {
                return Optional.empty();
            }
            
            // Get the target's field-relative pose
            var tagPose = targetPose.get();
            
            // Get the target's position relative to the camera
            double distance = getDistanceToTarget();
            double yaw = Math.toRadians(getTargetX());
            
            // Calculate the robot's position relative to the tag
            double xOffset = distance * Math.sin(yaw);
            double yOffset = distance * Math.cos(yaw);
            
            // Calculate the robot's field-relative pose
            // This is a simplified 2D approximation - for more accurate results,
            // consider using the Limelight's 3D pose estimation
            double robotX = tagPose.getX() - xOffset;
            double robotY = tagPose.getY() - yOffset;
            double robotRotation = tagPose.getRotation().getZ() - yaw + Math.PI;
            
            return Optional.of(new Pose2d(robotX, robotY, new Rotation2d(robotRotation)));
            
        } catch (Exception e) {
            DriverStation.reportError("Failed to get robot pose: " + e.getMessage(), false);
            return Optional.empty();
        }
    }

    @Override
    public void periodic() {
        // Update any periodic tasks here
        // For example, log the current target information
        if (hasTarget()) {
            // Log target information if needed
            // SmartDashboard.putNumber("AprilTag/ID", getTargetId());
            // SmartDashboard.putNumber("AprilTag/Distance", getDistanceToTarget());
        }
    }
}
