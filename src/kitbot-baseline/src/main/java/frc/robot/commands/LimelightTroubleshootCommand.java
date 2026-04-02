// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimelightTroubleshooter;

/**
 * Command that runs Limelight diagnostics and outputs troubleshooting information.
 */
public class LimelightTroubleshootCommand extends Command {
    
    private boolean completed = false;
    
    /**
     * Creates a new LimelightTroubleshootCommand.
     */
    public LimelightTroubleshootCommand() {
        // No requirements needed
    }
    
    @Override
    public void initialize() {
        completed = false;
        
        // Run comprehensive diagnostics
        String report = LimelightTroubleshooter.runDiagnostics();
        
        // Output to DriverStation console
        DriverStation.reportWarning("=== LIMELIGHT TROUBLESHOOTING REPORT ===", false);
        
        // Split report into lines and output each
        String[] lines = report.split("\n");
        for (String line : lines) {
            if (!line.trim().isEmpty()) {
                DriverStation.reportWarning(line, false);
            }
        }
        
        DriverStation.reportWarning("=== END TROUBLESHOOTING REPORT ===", false);
        
        completed = true;
    }
    
    @Override
    public void execute() {
        // Nothing to do - work done in initialize
    }
    
    @Override
    public void end(boolean interrupted) {
        // Command cleanup
    }
    
    @Override
    public boolean isFinished() {
        return completed;
    }
}
