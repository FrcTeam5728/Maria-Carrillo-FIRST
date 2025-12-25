package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class AutonLoader {
    
    public static class AutonStep {
        private final String commandName;
        private final JSONObject parameters;

        public AutonStep(String commandName, JSONObject parameters) {
            this.commandName = commandName;
            this.parameters = parameters;
        }

        public String commandName() {
            return commandName;
        }

        public JSONObject parameters() {
            return parameters;
        }
    }

    public static List<AutonStep> loadPlan(String fileName) {
        List<AutonStep> steps = new ArrayList<>();
        JSONParser parser = new JSONParser();

        try (FileReader reader = new FileReader(Filesystem.getDeployDirectory().toPath().resolve(fileName).toFile())) {
            // Parse the JSON file
            JSONArray stepsArray = (JSONArray) parser.parse(reader);
            
            // Convert each step to an AutonStep
            for (Object stepObj : stepsArray) {
                JSONObject stepJson = (JSONObject) stepObj;
                String command = (String) stepJson.get("command");
                JSONObject params = (JSONObject) stepJson.get("parameters");
                
                steps.add(new AutonStep(command, params));
            }
        } catch (IOException | ParseException e) {
            System.err.println("Error loading autonomous plan: " + e.getMessage());
            e.printStackTrace();
        }

        return steps;
    }
}
