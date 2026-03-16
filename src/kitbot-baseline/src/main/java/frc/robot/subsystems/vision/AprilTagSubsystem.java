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
    private final AprilTagMemory memory;
    private AprilTagEnvironment environment;
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
        this.memory = new AprilTagMemory(2.0, 8.0); // Remember for 2 seconds, up to 8 meters
        this.environment = AprilTagEnvironment.COMPETITION; // Default to competition mode
        
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
            // Load the default AprilTag field layout (use this to avoid references to non-existent constants)
            aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
            System.out.println("Loaded default AprilTag field layout with " + aprilTagFieldLayout.getTags().size() + " tags");
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
        
        // Add debugging for Limelight connection
        System.out.println("Limelight subsystem initialized with table: " + limelightTable.getPath());
        testLimelightConnection();
    }
    
    /**
     * Tests the Limelight connection and logs status information.
     */
    private void testLimelightConnection() {
        try {
            // Test if we can get values from the Limelight
            double tvValue = tv.getDouble(-1);
            double txValue = tx.getDouble(999);
            double tyValue = ty.getDouble(999);
            double tidValue = tid.getDouble(-1);
            
            System.out.println("=== LIMELIGHT CONNECTION TEST ===");
            System.out.println("Table path: " + limelightTable.getPath());
            System.out.println("tv (valid target): " + tvValue);
            System.out.println("tx (horizontal offset): " + txValue);
            System.out.println("ty (vertical offset): " + tyValue);
            System.out.println("tid (target ID): " + tidValue);
            
            // Check if values are reasonable
            if (tvValue == -1 && txValue == 999 && tyValue == 999 && tidValue == -1) {
                System.err.println("WARNING: Limelight may not be connected or configured!");
                System.err.println("Check:");
                System.err.println("1. Limelight is powered on (LED should be on)");
                System.err.println("2. USB cable is securely connected");
                System.err.println("3. Limelight is configured with name 'limelight'");
                System.err.println("4. NetworkTables are working");
            } else {
                System.out.println("Limelight connection appears to be working");
            }
            System.out.println("=================================");
        } catch (Exception e) {
            System.err.println("ERROR testing Limelight connection: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Checks if the Limelight is connected and responding.
     * 
     * @return true if Limelight appears to be connected, false otherwise
     */
    public boolean isLimelightConnected() {
        try {
            // Try to get a value and check if it's not the default error value
            double testValue = tx.getDouble(999);
            return testValue != 999;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Updates SmartDashboard with current Limelight status and target information.
     */
    private void updateSmartDashboard() {
        try {
            // Connection status
            boolean connected = isLimelightConnected();
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Limelight/Connected", connected);
            
            // Target detection
            boolean hasTarget = hasTarget();
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget);
            
            if (hasTarget) {
                // Target information
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/TargetID", getTargetId());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/HorizontalOffset", getTargetX());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/VerticalOffset", getTargetY());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/TargetArea", getTargetArea());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/Distance", getDistanceToTarget());
                
                // Robot pose if available
                Optional<Pose2d> robotPose = getRobotPose();
                if (robotPose.isPresent()) {
                    Pose2d pose = robotPose.get();
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/RobotX", pose.getX());
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/RobotY", pose.getY());
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/RobotRotation", pose.getRotation().getDegrees());
                }
            } else {
                // Clear target values when no target
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/TargetID", -1);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/HorizontalOffset", 0);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/VerticalOffset", 0);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/TargetArea", 0);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Limelight/Distance", 0);
            }
            
            // Environment mode
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Limelight/Environment", environment.getDisplayName());
            
        } catch (Exception e) {
            // Don't let SmartDashboard errors crash the subsystem
            System.err.println("Error updating SmartDashboard: " + e.getMessage());
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
    
    /**
     * Gets the field-relative pose of the currently detected AprilTag.
     * 
     * @return The AprilTag's field pose, or empty if no valid target is detected
     */
    public Optional<Pose2d> getTargetPose() {
        if (!hasTarget()) return Optional.empty();
        
        try {
            int tagId = getTargetId();
            var targetPose3d = aprilTagFieldLayout.getTagPose(tagId);
            
            if (targetPose3d.isEmpty()) {
                return Optional.empty();
            }
            
            // Convert Pose3d to Pose2d by taking only the X, Y coordinates and rotation around Z axis
            var pose3d = targetPose3d.get();
            var pose2d = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
            return Optional.of(pose2d);
            
        } catch (Exception e) {
            DriverStation.reportError("Failed to get target pose: " + e.getMessage(), false);
            return Optional.empty();
        }
    }

    @Override
    public void periodic() {
        // Add periodic connection status check (every 5 seconds)
        if (System.currentTimeMillis() % 5000 < 20) { // Roughly every 5 seconds
            if (!isLimelightConnected()) {
                System.err.println("Limelight not responding - check connection");
            }
        }
        
        // Update SmartDashboard with Limelight status
        updateSmartDashboard();
        
        // Update memory with current target data
        if (hasTarget()) {
            int tagId = getTargetId();
            Optional<Pose2d> targetPose = getTargetPose();
            double distance = getDistanceToTarget();
            double yaw = getTargetX(); // tx value
            
            if (targetPose.isPresent()) {
                // Store in memory regardless of environment mode
                memory.updateTarget(tagId, targetPose.get(), distance, yaw);
                
                // In competition mode, update odometry if we have field localization
                if (environment == AprilTagEnvironment.COMPETITION) {
                    Optional<Pose2d> robotPose = getRobotPose();
                    if (robotPose.isPresent()) {
                        // Update drive subsystem odometry here if needed
                        // This would require access to the drive subsystem
                        // For now, we just log the pose
                        System.out.println("Competition mode - Robot pose: " + robotPose.get());
                    }
                } else {
                    // Foreign mode - just store positions without field localization
                    System.out.println("Foreign mode - Remembered tag " + tagId + " at relative position");
                }
            }
        }
        
        // For example, log the current target information
        if (hasTarget()) {
            // Log target information if needed
            // SmartDashboard.putNumber("AprilTag/ID", getTargetId());
            // SmartDashboard.putNumber("AprilTag/Distance", getDistanceToTarget());
        }
    }
    
    /**
     * Gets the best remembered target (either current or from memory).
     * 
     * @return Optional containing the best target data
     */
    public Optional<AprilTagMemory.BestTarget> getBestTarget() {
        // If we have a current target, prefer it
        if (hasTarget()) {
            int tagId = getTargetId();
            Optional<Pose2d> targetPose = getTargetPose();
            double distance = getDistanceToTarget();
            double yaw = getTargetX();
            
            if (targetPose.isPresent()) {
                // Create a BestTarget with current data
                AprilTagMemory.BestTarget currentTarget = AprilTagMemory.BestTarget.create(
                    tagId, targetPose.get(), distance, yaw, 1.0, 1.0
                );
                return Optional.of(currentTarget);
            }
        }
        
        // Fall back to remembered targets
        return memory.getBestRememberedTarget();
    }
    
    /**
     * Gets a specific remembered target by ID.
     * 
     * @param tagId The AprilTag ID
     * @return Optional containing the remembered target data
     */
    public Optional<AprilTagMemory.TargetData> getRememberedTarget(int tagId) {
        return memory.getRememberedTarget(tagId);
    }
    
    /**
     * Clears all AprilTag memory.
     */
    public void clearMemory() {
        memory.clearMemory();
    }
    
    /**
     * Gets the number of remembered targets.
     * 
     * @return Number of targets in memory
     */
    public int getRememberedTargetCount() {
        return memory.getRememberedTargetCount();
    }
    
    /**
     * Gets the current environment mode.
     * 
     * @return Current environment (COMPETITION or FOREIGN)
     */
    public AprilTagEnvironment getEnvironment() {
        return environment;
    }
    
    /**
     * Sets the environment mode.
     * 
     * @param environment The new environment mode
     */
    public void setEnvironment(AprilTagEnvironment environment) {
        if (this.environment != environment) {
            System.out.println("Switching AprilTag environment from " + 
                this.environment.getDisplayName() + " to " + environment.getDisplayName());
            
            // Clear memory when switching modes to avoid confusion
            if (environment == AprilTagEnvironment.FOREIGN) {
                System.out.println("Clearing field localization data for foreign mode");
                memory.clearMemory();
            }
            
            this.environment = environment;
        }
    }
    
    /**
     * Toggles between competition and foreign mode.
     */
    public void toggleEnvironment() {
        setEnvironment(environment.getOpposite());
    }
    
    /**
     * Checks if currently in competition mode.
     * 
     * @return true if in competition mode
     */
    public boolean isCompetitionMode() {
        return environment == AprilTagEnvironment.COMPETITION;
    }
    
    /**
     * Checks if currently in foreign mode.
     * 
     * @return true if in foreign mode
     */
    public boolean isForeignMode() {
        return environment == AprilTagEnvironment.FOREIGN;
    }
    
    /**
     * Gets robot pose only if in competition mode.
     * In foreign mode, this will always return empty.
     * 
     * @return Optional containing robot pose if in competition mode and target is visible
     */
    public Optional<Pose2d> getCompetitionRobotPose() {
        if (environment == AprilTagEnvironment.COMPETITION) {
            return getRobotPose();
        }
        return Optional.empty();
    }
}
