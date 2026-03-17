// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.subsystems.CameraServerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PulsingShooterSubsystem;
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
    
    // Field position widgets
    private SimpleWidget fieldX;
    private SimpleWidget fieldY;
    private SimpleWidget fieldConfidence;
    private SimpleWidget positionSource;
    
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
        robotX = driverTab.add("Robot X", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -5, "max", 15))
            .withPosition(4, 3)
            .withSize(4, 1);
            
        robotY = driverTab.add("Robot Y", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -2, "max", 10))
            .withPosition(4, 4)
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
    public void updateCameraWidgets(CameraServerSubsystem cameraServer) {
        usbCameraStatus.getEntry().setBoolean(cameraServer.isUsbCameraAvailable());
        limelightCameraStatus.getEntry().setBoolean(cameraServer.isLimelightStreamAvailable());
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
        CameraServerSubsystem cameraServer,
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
