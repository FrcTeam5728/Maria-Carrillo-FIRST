// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.FieldPositionSystem;

/**
 * Command to reset the robot's field position.
 * Useful for resetting odometry when robot is in known position.
 */
public class ResetFieldPositionCommand extends Command {
    
    private final FieldPositionSystem fieldPositionSystem;
    private boolean completed = false;
    
    /**
     * Creates a new ResetFieldPositionCommand.
     * 
     * @param fieldPositionSystem The field position system to reset
     */
    public ResetFieldPositionCommand(FieldPositionSystem fieldPositionSystem) {
        this.fieldPositionSystem = fieldPositionSystem;
        // FieldPositionSystem is not a Subsystem, so no requirements needed
    }
    
    @Override
    public void initialize() {
        completed = false;
    }
    
    @Override
    public void execute() {
        // Reset position to field center
        fieldPositionSystem.resetPosition();
        completed = true;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Field position reset complete
    }
    
    @Override
    public boolean isFinished() {
        return completed;
    }
}
