// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The SubsystemManager handles all subsystems and their commands
    private final SubsystemManager subsystemManager = SubsystemManager.getInstance();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        
        // Register any named commands for PathPlanner
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        
        // Load button configurations from file
        try {
            CommandManager commandManager = CommandManager.getInstance(
                SubsystemManager.getInstance().getDrivebase(),
                SubsystemManager.getInstance().getDriverController()
            );
            commandManager.loadButtonConfigs("button_config.json");
        } catch (Exception e) {
            System.err.println("Failed to load button configurations: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Configure the button bindings for the robot.
     */
    private void configureBindings() {
        // Delegate button binding configuration to the SubsystemManager
        subsystemManager.configureButtonBindings();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return subsystemManager.getAutonomousCommand();
    }

    /**
     * Enable or disable motor braking.
     *
     * @param brake true to enable motor braking, false to disable
     */
    public void setMotorBrake(boolean brake) {
        subsystemManager.setMotorBrake(brake);
    }
}
}