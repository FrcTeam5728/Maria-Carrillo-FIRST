// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.DashboardLogger;

/**
 * Simple command to test Limelight connectivity.
 * Tests NetworkTables connection and basic functionality.
 */
public class LimelightTestCommand extends Command {
    
    private final LimelightSubsystem limelightSubsystem;
    private boolean completed = false;
    
    /**
     * Creates a new LimelightTestCommand.
     * 
     * @param limelightSubsystem The Limelight subsystem to test
     */
    public LimelightTestCommand(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(limelightSubsystem);
    }
    
    @Override
    public void initialize() {
        DashboardLogger.putString("Limelight/TestStatus", "=== LIMELIGHT CONNECTION TEST ===");
        completed = false;
    }
    
    @Override
    public void execute() {
        // Test basic connectivity
        boolean connected = limelightSubsystem.isConnected();
        boolean hasTarget = limelightSubsystem.hasTarget();
        
    DashboardLogger.putString("Limelight/ConnectionStatus", "");
    DashboardLogger.putBoolean("Limelight/Connected", connected);
    DashboardLogger.putBoolean("Limelight/HasTarget", hasTarget);
    DashboardLogger.putNumber("Limelight/TargetID", limelightSubsystem.getTargetId());
    DashboardLogger.putNumber("Limelight/TX", limelightSubsystem.getHorizontalOffset());
    DashboardLogger.putNumber("Limelight/TY", limelightSubsystem.getVerticalOffset());
    DashboardLogger.putNumber("Limelight/TargetArea", limelightSubsystem.getTargetArea());
    DashboardLogger.putNumber("Limelight/Distance", limelightSubsystem.getDistance());
    DashboardLogger.putNumber("Limelight/Latency", limelightSubsystem.getLatency());
        
        if (connected) {
            DashboardLogger.putString("Limelight/StatusMessage", "connected");
            DashboardLogger.putBoolean("Limelight/HasTarget", hasTarget);
        } else {
            DashboardLogger.putString("Limelight/StatusMessage", "not connected - check physical connection");
        }
        
        completed = true;
    }
    
    @Override
    public void end(boolean interrupted) {
        DashboardLogger.putString("Limelight/TestStatus", "=== LIMELIGHT TEST COMPLETE ===");
    }
    
    @Override
    public boolean isFinished() {
        return completed;
    }
}
