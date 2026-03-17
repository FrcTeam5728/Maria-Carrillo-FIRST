// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ShootingPositionManager;

/**
 * Command to select a shooting position using D-pad.
 * Provides different selection modes based on D-pad direction.
 */
public class SelectShootingPositionCommand extends Command {
    
    private final ShootingPositionManager positionManager;
    private final int direction; // 0=up, 90=right, 180=down, 270=left
    private boolean completed = false;
    
    /**
     * Creates a new SelectShootingPositionCommand.
     * 
     * @param positionManager Shooting position manager
     * @param direction D-pad direction (0=up, 90=right, 180=down, 270=left)
     */
    public SelectShootingPositionCommand(ShootingPositionManager positionManager, int direction) {
        this.positionManager = positionManager;
        this.direction = direction;
    }
    
    @Override
    public void initialize() {
        switch (direction) {
            case 0: // Up - Speaker positions
                positionManager.selectPositionByDirection(0);
                System.out.println("D-pad Up: " + positionManager.getCurrentPosition().getName());
                break;
            case 90: // Right - Next position
                positionManager.selectNextPosition();
                System.out.println("D-pad Right: " + positionManager.getCurrentPosition().getName());
                break;
            case 180: // Down - Stage positions
                positionManager.selectPositionByDirection(180);
                System.out.println("D-pad Down: " + positionManager.getCurrentPosition().getName());
                break;
            case 270: // Left - Previous position
                positionManager.selectPreviousPosition();
                System.out.println("D-pad Left: " + positionManager.getCurrentPosition().getName());
                break;
        }
        completed = true;
    }
    
    @Override
    public boolean isFinished() {
        return completed;
    }
}
