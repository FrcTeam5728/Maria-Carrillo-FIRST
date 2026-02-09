package frc.robot.config;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * Represents a button configuration that maps a physical button to a command.
 */
public class ButtonConfig {
    private final int port;
    private final int button;
    private final String command;
    private final boolean whenPressed;
    private final boolean whileHeld;
    private final boolean whenReleased;

    @JsonCreator
    public ButtonConfig(
            @JsonProperty("port") int port,
            @JsonProperty("button") int button,
            @JsonProperty("command") String command,
            @JsonProperty(value = "whenPressed", defaultValue = "false") boolean whenPressed,
            @JsonProperty(value = "whileHeld", defaultValue = "false") boolean whileHeld,
            @JsonProperty(value = "whenReleased", defaultValue = "false") boolean whenReleased) {
        this.port = port;
        this.button = button;
        this.command = command;
        this.whenPressed = whenPressed;
        this.whileHeld = whileHeld;
        this.whenReleased = whenReleased;
    }

    public int getPort() { return port; }
    public int getButton() { return button; }
    public String getCommand() { return command; }
    public boolean isWhenPressed() { return whenPressed; }
    public boolean isWhileHeld() { return whileHeld; }
    public boolean isWhenReleased() { return whenReleased; }
}
