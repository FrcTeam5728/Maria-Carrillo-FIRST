package frc.robot.config;

import org.json.JSONObject;

public class ButtonConfig {
    public enum CommandType {
        WHEN_PRESSED,
        WHEN_RELEASED,
        WHILE_HELD,
        TOGGLE_WHEN_PRESSED
    }

    public final String button;
    public final CommandType commandType;
    public final String command;

    public ButtonConfig(String button, CommandType commandType, String command) {
        this.button = button;
        this.commandType = commandType;
        this.command = command;
    }

    public static ButtonConfig fromJson(JSONObject json) {
        return new ButtonConfig(
            json.getString("button"),
            CommandType.valueOf(json.getString("commandType")),
            json.getString("command")
        );
    }
}
