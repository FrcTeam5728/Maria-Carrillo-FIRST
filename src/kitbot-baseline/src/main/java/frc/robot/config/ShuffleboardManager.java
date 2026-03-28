// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SimpleCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.utils.FieldPositionSystem;
import frc.robot.utils.ShootingPositionManager;
import java.util.Map;

/**
 * Manages Shuffleboard layout and widgets for driver station display.
 * Provides organized tabs for different robot systems and debugging.
 */
public class ShuffleboardManager {
    
    // Shuffleboard tabs
    private final ShuffleboardTab driverTab;
    private final ShuffleboardTab debugTab;
    private final ShuffleboardTab cameraTab;
    
    // Camera widgets
    private SimpleWidget usbCameraStatus;
    private SimpleWidget limelightCameraStatus;
    private SimpleWidget limelightUrl;
    
    // Limelight widgets
    private SimpleWidget limelightConnected;
    private SimpleWidget limelightHasTarget;
    private SimpleWidget limelightHorizontalOffset;
    private SimpleWidget limelightVerticalOffset;
    private SimpleWidget limelightDistance;
    private SimpleWidget limelightStatus;
    
    // Odometry widgets
    private SimpleWidget robotX;
    private SimpleWidget robotY;
    private SimpleWidget robotHeading;
    private SimpleWidget gyroAngle;
    private SimpleWidget leftEncoder;
    private SimpleWidget rightEncoder;
    
    // Field visualization widget
    private Field2d fieldWidget;
    
    // Field position widgets
    private SimpleWidget fieldX;
    private SimpleWidget fieldY;
    private SimpleWidget fieldConfidence;
    private SimpleWidget positionSource;
    
    // Advanced positioning widgets
    private SimpleWidget advancedPoseX;
    private SimpleWidget advancedPoseY;
    private SimpleWidget advancedPoseHeading;
    private SimpleWidget tagsDetected;
    private SimpleWidget positioningConfidence;
    
    // Shooting widgets
    private SimpleWidget selectedPosition;
    private SimpleWidget preferredDistance;
    private SimpleWidget shootingConfidence;
    
    // Fuel/shooter widgets
    private SimpleWidget intakeRunning;
    private SimpleWidget feederRunning;
    private SimpleWidget shooterActive;
    private SimpleWidget shooterSpeed;
    
    /**
     * Creates a new ShuffleboardManager.
     */
    public ShuffleboardManager() {
        // Create tabs
        driverTab = Shuffleboard.getTab("Driver");
        debugTab = Shuffleboard.getTab("Debug");
        cameraTab = Shuffleboard.getTab("Cameras");
        
        // Initialize widgets
        initializeCameraWidgets();
        initializeLimelightWidgets();
        initializeOdometryWidgets();
        initializeFieldPositionWidgets();
        initializeShootingWidgets();
        initializeFuelWidgets();
        initializeDebugWidgets();
        
        System.out.println("ShuffleboardManager initialized with organized tabs");
    }
    
    /**
     * Initializes camera-related widgets.
     */
    private void initializeCameraWidgets() {
        usbCameraStatus = cameraTab.add("USB Camera Available", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(2, 1);
            
        limelightCameraStatus = cameraTab.add("Limelight Available", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 0)
            .withSize(2, 1);
            
        limelightUrl = cameraTab.add("Limelight URL", "http://10.57.28.11:5800/stream.mjpg")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(4, 1);
    }
    
    /**
     * Initializes Limelight vision widgets.
     */
    private void initializeLimelightWidgets() {
        limelightConnected = driverTab.add("Limelight Connected", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(2, 1);
            
        limelightHasTarget = driverTab.add("Target Found", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(2, 0)
            .withSize(2, 1);
            
        limelightHorizontalOffset = driverTab.add("Horizontal Offset", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -27, "max", 27))
            .withPosition(0, 1)
            .withSize(4, 1);
            
        limelightVerticalOffset = driverTab.add("Vertical Offset", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -20.5, "max", 20.5))
            .withPosition(0, 2)
            .withSize(4, 1);
            
        limelightDistance = driverTab.add("Distance (m)", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 10))
            .withPosition(0, 3)
            .withSize(4, 1);
            
        limelightStatus = driverTab.add("Limelight Status", "Unknown")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 4)
            .withSize(4, 1);
    }
    
    /**
     * Initializes odometry and drive system widgets.
     */
    private void initializeOdometryWidgets() {
        // Create field visualization widget
        fieldWidget = new Field2d();
        driverTab.add("Field", fieldWidget)
            .withPosition(4, 0)
            .withSize(4, 3);
        
        robotX = driverTab.add("Robot X", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 15))
            .withPosition(8, 3)
            .withSize(4, 1);
            
        robotY = driverTab.add("Robot Y", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -2, "max", 10))
            .withPosition(8, 4)
            .withSize(4, 1);
            
        robotHeading = driverTab.add("Robot Heading", 0.0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(4, 5)
            .withSize(2, 3);
            
        gyroAngle = driverTab.add("Gyro Angle", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -180, "max", 180))
            .withPosition(6, 5)
            .withSize(2, 1);
            
        leftEncoder = driverTab.add("Left Encoder", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -10, "max", 10))
            .withPosition(8, 3)
            .withSize(4, 1);
            
        rightEncoder = driverTab.add("Right Encoder", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -10, "max", 10))
            .withPosition(8, 4)
            .withSize(4, 1);
    }
    
    /**
     * Initializes field position system widgets.
     */
    private void initializeFieldPositionWidgets() {
        fieldX = driverTab.add("Field X", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(8, 5)
            .withSize(2, 1);
            
        fieldY = driverTab.add("Field Y", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(10, 5)
            .withSize(2, 1);
            
        fieldConfidence = driverTab.add("Field Confidence", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .withPosition(12, 5)
            .withSize(2, 1);
            
        positionSource = driverTab.add("Position Source", "UNKNOWN")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(8, 6)
            .withSize(6, 1);
            
        // Advanced positioning widgets
        advancedPoseX = driverTab.add("Advanced X", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 15))
            .withPosition(0, 6)
            .withSize(4, 1);
            
        advancedPoseY = driverTab.add("Advanced Y", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -2, "max", 10))
            .withPosition(4, 7)
            .withSize(4, 1);
            
        advancedPoseHeading = driverTab.add("Advanced Heading", 0.0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(0, 8)
            .withSize(2, 3);
            
        tagsDetected = driverTab.add("Tags Detected", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 8)
            .withSize(2, 1);
            
        positioningConfidence = driverTab.add("Positioning Confidence", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .withPosition(2, 9)
            .withSize(2, 1);
    }
    
    /**
     * Initializes shooting system widgets.
     */
    private void initializeShootingWidgets() {
        selectedPosition = driverTab.add("Selected Position", "None")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 5)
            .withSize(4, 1);
            
        preferredDistance = driverTab.add("Preferred Distance", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 10))
            .withPosition(0, 6)
            .withSize(4, 1);
            
        shootingConfidence = driverTab.add("Shooting Confidence", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 7)
            .withSize(4, 1);
    }
    
    /**
     * Initializes fuel system and shooter widgets.
     */
    private void initializeFuelWidgets() {
        intakeRunning = driverTab.add("Intake Running", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 0)
            .withSize(2, 1);
            
        feederRunning = driverTab.add("Feeder Running", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(6, 0)
            .withSize(2, 1);
            
        shooterActive = driverTab.add("Shooter Active", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 1)
            .withSize(2, 1);
            
        shooterSpeed = driverTab.add("Shooter Speed", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 1))
            .withPosition(6, 1)
            .withSize(2, 1);
    }
    
    /**
     * Initializes debug widgets.
     */
    private void initializeDebugWidgets() {
        debugTab.add("Debug Error", "None")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(6, 1);
            
        debugTab.add("Model Status", "Unknown")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 1)
            .withSize(6, 1);
            
        debugTab.add("Test Status", "Ready")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 2)
            .withSize(6, 1);
    }
    
    /**
     * Updates camera widgets with current status.
     */
    public void updateCameraWidgets(SimpleCameraSubsystem cameraServer) {
        usbCameraStatus.getEntry().setBoolean(cameraServer.isUsbCameraAvailable());
        limelightCameraStatus.getEntry().setBoolean(cameraServer.isLimelightAvailable());
        limelightUrl.getEntry().setString(cameraServer.getLimelightStreamUrl());
    }
    
    /**
     * Updates Limelight widgets with current data.
     */
    public void updateLimelightWidgets(LimelightSubsystem limelight) {
        limelightConnected.getEntry().setBoolean(limelight.isConnected());
        limelightHasTarget.getEntry().setBoolean(limelight.hasTarget());
        limelightHorizontalOffset.getEntry().setDouble(limelight.getHorizontalOffset());
        limelightVerticalOffset.getEntry().setDouble(limelight.getVerticalOffset());
        limelightDistance.getEntry().setDouble(limelight.getDistance());
    }
    
    /**
     * Updates odometry widgets with current data.
     */
    public void updateOdometryWidgets(DriveSubsystem drive) {
        var pose = drive.getPose();
        robotX.getEntry().setDouble(pose.getX());
        robotY.getEntry().setDouble(pose.getY());
        robotHeading.getEntry().setDouble(pose.getRotation().getDegrees());
        gyroAngle.getEntry().setDouble(drive.getGyroAngle());
        
        // Update field widget with robot position
        fieldWidget.setRobotPose(pose);
        
        // Encoder values would need to be accessed from DriveSubsystem
        // This is a placeholder - actual implementation may vary
        leftEncoder.getEntry().setDouble(0.0); // Placeholder
        rightEncoder.getEntry().setDouble(0.0); // Placeholder
    }
    
    /**
     * Updates field position widgets with current data.
     */
    public void updateFieldPositionWidgets(FieldPositionSystem fieldPosition) {
        var pose = fieldPosition.getRobotPose();
        fieldX.getEntry().setDouble(pose.getX());
        fieldY.getEntry().setDouble(pose.getY());
        fieldConfidence.getEntry().setDouble(fieldPosition.getConfidence());
        positionSource.getEntry().setString(fieldPosition.getPositionSource());
    }
    
    /**
     * Updates advanced positioning widgets with AprilTag/odometry data.
     */
    public void updateAdvancedPositioningWidgets(DriveSubsystem drive) {
        if (drive instanceof DriveSubsystemSparkMax) {
            var sparkDrive = (DriveSubsystemSparkMax) drive;
            var advancedPose = sparkDrive.getPositioning().getFieldPose();
            
            advancedPoseX.getEntry().setDouble(advancedPose.getX());
            advancedPoseY.getEntry().setDouble(advancedPose.getY());
            advancedPoseHeading.getEntry().setDouble(advancedPose.getRotation().getDegrees());
            tagsDetected.getEntry().setNumber(sparkDrive.getPositioning().hasRecentAprilTagDetection() ? 1 : 0);
            positioningConfidence.getEntry().setDouble(sparkDrive.getPositioning().getConfidence());
        }
    }
    
    /**
     * Displays a trajectory on the field widget.
     * Useful for visualizing PathPlanner paths or autonomous trajectories.
     * 
     * @param trajectory The trajectory to display
     * @param name The name of the trajectory (for legend)
     */
    public void displayTrajectory(Trajectory trajectory, String name) {
        if (fieldWidget != null && trajectory != null) {
            fieldWidget.getObject(name).setTrajectory(trajectory);
        }
    }
    
    /**
     * Displays a simple path on the field widget using a list of poses.
     * 
     * @param poses List of poses that make up the path
     * @param name The name of the path (for legend)
     */
    public void displayPath(Pose2d[] poses, String name) {
        if (fieldWidget != null && poses != null && poses.length > 0) {
            // Create a simple trajectory from the poses
            var trajectory = new Trajectory();
            var states = new java.util.ArrayList<Trajectory.State>();
            
            for (int i = 0; i < poses.length; i++) {
                var state = new Trajectory.State();
                state.poseMeters = poses[i];
                state.timeSeconds = i * 0.1; // 0.1 seconds between poses
                states.add(state);
            }
            
            trajectory.getStates().addAll(states);
            fieldWidget.getObject(name).setTrajectory(trajectory);
        }
    }
    
    /**
     * Clears all trajectories from the field widget except the robot pose.
     */
    public void clearTrajectories() {
        if (fieldWidget != null) {
            // Create a new field widget to clear all trajectories
            // This is the simplest way to clear all objects
            var currentPose = fieldWidget.getRobotPose();
            fieldWidget = new Field2d();
            fieldWidget.setRobotPose(currentPose);
            
            // Re-add the widget to Shuffleboard
            driverTab.add("Field", fieldWidget)
                .withPosition(4, 0)
                .withSize(4, 3);
        }
    }
    
    /**
     * Gets the field widget for direct manipulation.
     * 
     * @return The Field2d widget
     */
    public Field2d getFieldWidget() {
        return fieldWidget;
    }
    
    /**
     * Updates shooting widgets with current data.
     */
    public void updateShootingWidgets(ShootingPositionManager shootingManager) {
        var currentPosition = shootingManager.getCurrentPosition();
        selectedPosition.getEntry().setString(currentPosition.getName());
        preferredDistance.getEntry().setDouble(currentPosition.getPreferredDistance());
        
        // Get shooting parameters for confidence
        double[] params = shootingManager.getShootingParameters();
        shootingConfidence.getEntry().setDouble(params[2]);
    }
    
    /**
     * Updates all widgets with current subsystem data.
     */
    public void updateAllWidgets(
        SimpleCameraSubsystem cameraServer,
        LimelightSubsystem limelight,
        DriveSubsystem drive,
        FieldPositionSystem fieldPosition,
        ShootingPositionManager shootingManager
    ) {
        updateCameraWidgets(cameraServer);
        updateLimelightWidgets(limelight);
        updateOdometryWidgets(drive);
        updateFieldPositionWidgets(fieldPosition);
        updateShootingWidgets(shootingManager);
    }
    
    /**
     * Gets the driver tab for additional customization.
     */
    public ShuffleboardTab getDriverTab() {
        return driverTab;
    }
    
    /**
     * Gets the debug tab for additional customization.
     */
    public ShuffleboardTab getDebugTab() {
        return debugTab;
    }
    
    /**
     * Gets the camera tab for additional customization.
     */
    public ShuffleboardTab getCameraTab() {
        return cameraTab;
    }
}
