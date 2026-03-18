package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Abstract base class for fuel/intake subsystems.
 * Provides a common interface and shared implementation for different motor controller implementations.
 * 
 * This class handles SmartDashboard initialization, provides default command factories,
 * and implements the core operation logic. Subclasses only need to implement motor control.
 */
public abstract class FuelSubsystem extends SubsystemBase {
    /**
     * Initializes SmartDashboard entries for fuel subsystem tuning.
     * Should be called by subclass constructors after motor initialization.
     */
    protected void initializeDashboard() {
        SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
        SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
        SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
        SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
        SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    }

    /**
     * Gets the voltage value from the dashboard for a given operation.
     * 
     * @param dashboardKey The SmartDashboard key
     * @param defaultValue The default value if the key doesn't exist
     * @return The voltage value from the dashboard
     */
    protected double getDashboardVoltage(String dashboardKey, double defaultValue) {
        return SmartDashboard.getNumber(dashboardKey, defaultValue);
    }

    /**
     * Sets the feeder roller voltage.
     * Implemented by subclasses to set the appropriate motor controller voltage.
     * 
     * @param voltage The voltage to set
     */
    protected abstract void setFeederVoltage(double voltage);

    /**
     * Sets the intake/launcher roller voltage.
     * Implemented by subclasses to set the appropriate motor controller voltage.
     * 
     * @param voltage The voltage to set
     */
    protected abstract void setIntakeLauncherVoltage(double voltage);

    /**
     * Sets the rollers to values for intaking fuel.
     */
    public void intake() {
        setFeederVoltage(getDashboardVoltage("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
        setIntakeLauncherVoltage(getDashboardVoltage("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
    }

    /**
     * Sets the rollers to values for ejecting fuel back out the intake.
     */
    public void eject() {
        setFeederVoltage(-1 * getDashboardVoltage("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
        setIntakeLauncherVoltage(-1 * getDashboardVoltage("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
    }

    /**
     * Sets the rollers to values for launching fuel.
     */
    public void launch() {
        setFeederVoltage(getDashboardVoltage("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
        setIntakeLauncherVoltage(getDashboardVoltage("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    }

    /**
     * Stops all rollers.
     */
    public void stop() {
        setFeederVoltage(0);
        setIntakeLauncherVoltage(0);
    }

    /**
     * Spins up the launcher roller while spinning the feeder roller to push fuel away from the launcher.
     */
    public void spinUp() {
        setFeederVoltage(getDashboardVoltage("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
        setIntakeLauncherVoltage(getDashboardVoltage("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    }

    /**
     * Creates a command to spin up the launcher.
     * Provides a default implementation that can be overridden if needed.
     * 
     * @return Command that spins up the launcher
     */
    public Command spinUpCommand() {
        return this.run(() -> spinUp());
    }

    /**
     * Creates a command to launch fuel.
     * Provides a default implementation that can be overridden if needed.
     * 
     * @return Command that launches fuel
     */
    public Command launchCommand() {
        return this.run(() -> launch());
    }
}
