package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Wrapper class for Logitech Extreme 3D Pro joystick.
 * Provides convenient access to all axes and buttons.
 */
public class Extreme3DPro {
    private final Joystick joystick;

    // Button mappings
    public final JoystickButton trigger;
    public final JoystickButton thumbButton;
    public final JoystickButton button3;
    public final JoystickButton button4;
    public final JoystickButton button5;
    public final JoystickButton button6;
    public final JoystickButton button7;
    public final JoystickButton button8;
    public final JoystickButton button9;
    public final JoystickButton button10;
    public final JoystickButton button11;
    public final JoystickButton button12;
    
    // POV (hat switch) buttons
    public final POVButton povUp;
    public final POVButton povRight;
    public final POVButton povDown;
    public final POVButton povLeft;

    /**
     * Create a new Extreme 3D Pro joystick on a specific port.
     * @param port The USB port number (0-5)
     */
    public Extreme3DPro(int port) {
        joystick = new Joystick(port);
        
        // Initialize buttons
        trigger = new JoystickButton(joystick, 1);
        thumbButton = new JoystickButton(joystick, 2);
        button3 = new JoystickButton(joystick, 3);
        button4 = new JoystickButton(joystick, 4);
        button5 = new JoystickButton(joystick, 5);
        button6 = new JoystickButton(joystick, 6);
        button7 = new JoystickButton(joystick, 7);
        button8 = new JoystickButton(joystick, 8);
        button9 = new JoystickButton(joystick, 9);
        button10 = new JoystickButton(joystick, 10);
        button11 = new JoystickButton(joystick, 11);
        button12 = new JoystickButton(joystick, 12);
        
        // Initialize POV (hat switch) buttons
        povUp = new POVButton(joystick, 0);
        povRight = new POVButton(joystick, 90);
        povDown = new POVButton(joystick, 180);
        povLeft = new POVButton(joystick, 270);
    }

    // Axis getters with deadband
    public double getX() {
        return applyDeadband(joystick.getRawAxis(0));
    }
    
    public double getY() {
        return applyDeadband(joystick.getRawAxis(1));
    }
    
    public double getZ() {
        return applyDeadband(joystick.getRawAxis(2));
    }
    
    public double getThrottle() {
        // Throttle is inverted (pulled back = 1, pushed forward = -1)
        return (-joystick.getRawAxis(3) + 1) / 2; // Convert to 0-1 range
    }
    
    // Button getters
    public boolean getTrigger() {
        return joystick.getRawButton(1);
    }
    
    public boolean getThumbButton() {
        return joystick.getRawButton(2);
    }
    
    // POV (hat switch) getter
    public int getPOV() {
        return joystick.getPOV();
    }
    
    /**
     * Apply a deadband to joystick input.
     * @param value The input value
     * @param deadband The deadband threshold (default 0.1)
     * @return The adjusted value
     */
    private double applyDeadband(double value) {
        return applyDeadband(value, 0.1);
    }
    
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }
    
    /**
     * Get the underlying Joystick object for advanced functionality.
     * @return The underlying Joystick instance
     */
    public Joystick getJoystick() {
        return joystick;
    }
}
