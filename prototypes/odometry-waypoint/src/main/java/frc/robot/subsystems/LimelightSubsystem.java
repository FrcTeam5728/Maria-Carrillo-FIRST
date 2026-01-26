package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry tv;  // Whether the limelight has any valid targets (0 or 1)
    private final NetworkTableEntry tx;   // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    private final NetworkTableEntry ty;   // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    private final NetworkTableEntry ta;   // Target Area (0% of image to 100% of image)
    private final NetworkTableEntry ledMode;  // Sets limelight's LED state

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ledMode = table.getEntry("ledMode");
        
        // Initialize with LED off
        setLedMode(LedMode.OFF);
    }

    /**
     * @return Whether the limelight has any valid targets
     */
    public boolean hasTarget() {
        return tv.getDouble(0) == 1;
    }

    /**
     * @return Horizontal offset from crosshair to target (-27 to 27 degrees)
     */
    public double getTx() {
        return tx.getDouble(0.0);
    }

    /**
     * @return Vertical offset from crosshair to target (-20.5 to 20.5 degrees)
     */
    public double getTy() {
        return ty.getDouble(0.0);
    }

    /**
     * @return Target area (0% to 100% of image)
     */
    public double getTa() {
        return ta.getDouble(0.0);
    }

    /**
     * Sets the LED mode of the Limelight
     * @param mode The LED mode to set
     */
    public void setLedMode(LedMode mode) {
        ledMode.setNumber(mode.getValue());
    }

    /**
     * Toggles the Limelight's LED on or off
     */
    public void toggleLed() {
        if (ledMode.getDouble(1.0) == LedMode.ON.getValue()) {
            setLedMode(LedMode.OFF);
        } else {
            setLedMode(LedMode.ON);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * LED Modes for the Limelight
     */
    public enum LedMode {
        DEFAULT(0),  // Use the LED Mode set in the current pipeline
        OFF(1),      // Force LEDs off
        BLINK(2),    // Force LEDs to blink
        ON(3);       // Force LEDs on

        private final int value;

        LedMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
}
