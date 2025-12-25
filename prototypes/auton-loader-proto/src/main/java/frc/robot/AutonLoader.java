package frc.robot;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class AutonLoader {

    public static List<AutonStep> loadPlan(String filename) {
        JSONParser parser = new JSONParser();
        List<AutonStep> steps = new ArrayList<>();

        try (FileReader reader = new FileReader("/home/lvuser/deploy/autons/" + filename)) { // Correct file path needed
            JSONArray jsonArray = (JSONArray) parser.parse(reader);

            for (Object obj : jsonArray) {
                JSONObject jsonStep = (JSONObject) obj;
                String commandName = (String) jsonStep.get("command");
                JSONObject params = (JSONObject) jsonStep.get("parameters");
                
                steps.add(new AutonStep(commandName, params));
            }
        } catch (Exception e) {
            System.err.println("Error parsing autonomous file: " + e.getMessage());
            e.printStackTrace();
        }
        return steps;
    }
    
    // Simple record/class to hold a single step of the plan
    public record AutonStep(String commandName, JSONObject parameters) {} 
}