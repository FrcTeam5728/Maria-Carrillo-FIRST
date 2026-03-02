// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

/**
 * Environment mode for AprilTag processing.
 * Determines how AprilTag data is used based on the current environment.
 */
public enum AprilTagEnvironment {
    /**
     * Competition mode - Uses field layout for full robot localization.
     * Updates robot odometry to know exact position on the field.
     * Requires valid AprilTagFieldLayout and field coordinates.
     */
    COMPETITION("Competition", true),
    
    /**
     * Foreign mode - Stores tag positions without field map.
     * Only remembers relative positions of tags for basic tracking.
     * Does not update odometry or use field coordinates.
     */
    FOREIGN("Foreign", false);
    
    private final String displayName;
    private final boolean usesFieldLocalization;
    
    AprilTagEnvironment(String displayName, boolean usesFieldLocalization) {
        this.displayName = displayName;
        this.usesFieldLocalization = usesFieldLocalization;
    }
    
    /**
     * Gets the display name for this environment.
     */
    public String getDisplayName() {
        return displayName;
    }
    
    /**
     * Checks if this environment uses field localization.
     * 
     * @return true if competition mode (uses field layout), false if foreign mode
     */
    public boolean usesFieldLocalization() {
        return usesFieldLocalization;
    }
    
    /**
     * Gets the opposite environment mode.
     */
    public AprilTagEnvironment getOpposite() {
        return this == COMPETITION ? FOREIGN : COMPETITION;
    }
}
