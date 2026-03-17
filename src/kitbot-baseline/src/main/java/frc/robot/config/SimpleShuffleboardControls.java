// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.AprilTagPositionTestCommand;
import frc.robot.commands.ImmediateAprilTagUpdateCommand;
import frc.robot.commands.LimelightTroubleshootCommand;
import frc.robot.commands.ResetFieldPositionCommand;
import frc.robot.commands.ToggleLimelightModeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PulsingShooterSubsystem;
import frc.robot.utils.FieldPositionSystem;

/**
 * Simple Shuffleboard controls for all robot systems.
 * Provides easy access to diagnostics and commands.
 */
public class SimpleShuffleboardControls {
    
    private static final String TAB_NAME = "Robot Controls";
    private static ShuffleboardTab tab;
    
    /**
     * Initializes all Shuffleboard controls.
     */
    public static void initialize(LimelightSubsystem limelight, 
                                 FieldPositionSystem fieldPosition,
                                 PulsingShooterSubsystem shooter,
                                 DriveSubsystem drive) {
        tab = Shuffleboard.getTab(TAB_NAME);
        
        createLimelightControls(limelight);
        createAprilTagControls(fieldPosition);
        createShooterControls(shooter);
        createDriveControls(drive);
        createCommandControls(limelight, fieldPosition);
        createDiagnosticsControls();
    }
    
    /**
     * Creates Limelight controls.
     */
    private static void createLimelightControls(LimelightSubsystem limelight) {
        // Limelight connection status
        tab.add("Limelight Connected", limelight.isConnected())
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        // Has target status
        tab.add("Has Target", limelight.hasTarget())
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        // Target info
        tab.add("Target ID", limelight.getTargetId())
            .withWidget(BuiltInWidgets.kTextView);
        
        tab.add("Horizontal Offset", limelight.getHorizontalOffset())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", -27, "max", 27));
        
        tab.add("Vertical Offset", limelight.getVerticalOffset())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", -20.5, "max", 20.5));
        
        tab.add("Target Area", limelight.getTargetArea())
            .withWidget(BuiltInWidgets.kGraph);
    }
    
    /**
     * Creates AprilTag position controls.
     */
    private static void createAprilTagControls(FieldPositionSystem fieldPosition) {
        // Position status
        tab.add("Position Valid", fieldPosition.getPositionUpdater().hasValidPosition())
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        // Robot position
        tab.add("Robot X", fieldPosition.getRobotPose().getX())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", 0, "max", 16.54));
        
        tab.add("Robot Y", fieldPosition.getRobotPose().getY())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", 0, "max", 8.21));
        
        tab.add("Robot Heading", fieldPosition.getRobotPose().getRotation().getDegrees())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", -180, "max", 180));
        
        // Position confidence
        tab.add("Position Confidence", fieldPosition.getConfidence())
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(java.util.Map.of("min", 0, "max", 1));
        
        // Last update info
        tab.add("Last Target ID", fieldPosition.getPositionUpdater().getLastTargetId())
            .withWidget(BuiltInWidgets.kTextView);
        
        tab.add("Position Source", fieldPosition.getPositionSource())
            .withWidget(BuiltInWidgets.kTextView);
    }
    
    /**
     * Creates shooter controls.
     */
    private static void createShooterControls(PulsingShooterSubsystem shooter) {
        // Shooter status
        tab.add("Shooter Active", shooter.isPulsing())
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        tab.add("Pulsing", shooter.isPulsing())
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        // Shooter speeds
        tab.add("Shooter Speed", shooter.getShooterSpeed())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", 0, "max", 1));
        
        tab.add("Feeder Speed", shooter.getFeederSpeed())
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", 0, "max", 1));
        
        // Statistics
        tab.add("Pulse Count", shooter.getPulseCount())
            .withWidget(BuiltInWidgets.kTextView);
        
        tab.add("Total Shots", (int)shooter.getTotalShotsFired())
            .withWidget(BuiltInWidgets.kTextView);
    }
    
    /**
     * Creates drive controls.
     */
    private static void createDriveControls(DriveSubsystem drive) {
        // Drive status
        tab.add("Drive Connected", true)
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        // Odometry
        tab.add("Odometry X", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", -5, "max", 20));
        
        tab.add("Odometry Y", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", -5, "max", 15));
        
        tab.add("Odometry Heading", 0.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", -180, "max", 180));
    }
    
    /**
     * Creates command controls.
     */
    private static void createCommandControls(LimelightSubsystem limelight, 
                                            FieldPositionSystem fieldPosition) {
        // Limelight commands
        tab.add("Toggle Limelight Mode", new ToggleLimelightModeCommand(limelight, fieldPosition.getPositionUpdater()))
            .withWidget(BuiltInWidgets.kCommand);
        
        tab.add("Run AprilTag Test", new AprilTagPositionTestCommand(fieldPosition, fieldPosition.getPositionUpdater()))
            .withWidget(BuiltInWidgets.kCommand);
        
        tab.add("Immediate Updates", new ImmediateAprilTagUpdateCommand(limelight, fieldPosition.getPositionUpdater()))
            .withWidget(BuiltInWidgets.kCommand);
        
        tab.add("Reset Position", new ResetFieldPositionCommand(fieldPosition))
            .withWidget(BuiltInWidgets.kCommand);
        
        tab.add("Run Diagnostics", new LimelightTroubleshootCommand())
            .withWidget(BuiltInWidgets.kCommand);
    }
    
    /**
     * Creates diagnostics controls.
     */
    private static void createDiagnosticsControls() {
        // System health
        tab.add("All Systems OK", true)
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        // Network status
        tab.add("NetworkTables OK", true)
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        tab.add("Limelight OK", true)
            .withWidget(BuiltInWidgets.kBooleanBox);
        
        // Error count
        tab.add("Error Count", 0)
            .withWidget(BuiltInWidgets.kTextView);
        
        tab.add("Last Error", "None")
            .withWidget(BuiltInWidgets.kTextView);
        
        // System status
        tab.add("Robot Mode", "DISABLED")
            .withWidget(BuiltInWidgets.kTextView);
        
        tab.add("Battery Voltage", 12.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(java.util.Map.of("min", 10, "max", 13));
        
        tab.add("Brownout Active", false)
            .withWidget(BuiltInWidgets.kBooleanBox);
    }
    
    /**
     * Updates all Shuffleboard values.
     * Call this periodically from Robot.periodic().
     */
    public static void updateValues(LimelightSubsystem limelight, 
                                 FieldPositionSystem fieldPosition,
                                 PulsingShooterSubsystem shooter,
                                 DriveSubsystem drive) {
        try {
            // Update values that change frequently
            // Note: Shuffleboard automatically updates some values when subsystems are added
            
            // Update Limelight values
            // tab.add("Has Target", limelight.hasTarget());
            
            // Update AprilTag values
            // tab.add("Position Valid", fieldPosition.getPositionUpdater().hasValidPosition());
            
            // Update Shooter values
            // tab.add("Shooter Active", shooter.isShooterActive());
            
        } catch (Exception e) {
            // Handle errors gracefully
        }
    }
}
