// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.field;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Utility class for looking up POI data from the JSON file.
 * Provides methods to find POIs by AprilTag ID and get absolute positions.
 */
public class POILookup {
    
    private static POILookup instance;
    private final Map<Integer, POIData> aprilTagMap;
    private final Map<String, POIData> poiMap;
    
    /**
     * POI data structure.
     */
    public static class POIData {
        public final String id;
        public final String type;
        public final Pose2d position;
        public final double value;
        public final double confidence;
        public final double recommendedDistance;
        public final double interactionRadius;
        public final boolean active;
        public final String alliance;
        public final int[] aprilTagIds;
        
        public POIData(String id, String type, Pose2d position, double value, 
                      double confidence, double recommendedDistance, 
                      double interactionRadius, boolean active, 
                      String alliance, int[] aprilTagIds) {
            this.id = id;
            this.type = type;
            this.position = position;
            this.value = value;
            this.confidence = confidence;
            this.recommendedDistance = recommendedDistance;
            this.interactionRadius = interactionRadius;
            this.active = active;
            this.alliance = alliance;
            this.aprilTagIds = aprilTagIds;
        }
    }
    
    private POILookup() {
        this.aprilTagMap = new HashMap<>();
        this.poiMap = new HashMap<>();
        loadPOIData();
    }
    
    /**
     * Gets the singleton instance.
     * 
     * @return POILookup instance
     */
    public static POILookup getInstance() {
        if (instance == null) {
            instance = new POILookup();
        }
        return instance;
    }
    
    /**
     * Loads POI data from the JSON file.
     */
    private void loadPOIData() {
        try {
            File poisFile = new File(Filesystem.getDeployDirectory(), "field/pois.json");
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(poisFile);
            
            JsonNode pois = root.get("field").get("pois");
            for (JsonNode poi : pois) {
                // Extract POI data
                String id = poi.get("id").asText();
                String type = poi.get("type").asText();
                JsonNode pos = poi.get("position");
                double x = pos.get("x").asDouble();
                double y = pos.get("y").asDouble();
                double value = poi.get("value").asDouble();
                double confidence = poi.get("confidence").asDouble();
                double recommendedDistance = poi.get("recommended_distance").asDouble();
                double interactionRadius = poi.get("interaction_radius").asDouble();
                boolean active = poi.get("active").asBoolean();
                String alliance = poi.get("alliance").asText();
                
                // Extract AprilTag IDs
                int[] aprilTagIds = new int[0];
                if (poi.has("april_tag_ids") && poi.get("april_tag_ids").isArray()) {
                    JsonNode tagsArray = poi.get("april_tag_ids");
                    aprilTagIds = new int[tagsArray.size()];
                    for (int i = 0; i < tagsArray.size(); i++) {
                        aprilTagIds[i] = tagsArray.get(i).asInt();
                    }
                }
                
                // Create POI data
                POIData poiData = new POIData(
                    id, type, new Pose2d(x, y, new Rotation2d()),
                    value, confidence, recommendedDistance, 
                    interactionRadius, active, alliance, aprilTagIds
                );
                
                // Add to maps
                poiMap.put(id, poiData);
                for (int tagId : aprilTagIds) {
                    aprilTagMap.put(tagId, poiData);
                }
            }
            
        } catch (IOException e) {
            System.err.println("Failed to load POI data: " + e.getMessage());
        }
    }
    
    /**
     * Gets POI data by AprilTag ID.
     * 
     * @param aprilTagId AprilTag ID to look up
     * @return POI data, or null if not found
     */
    public POIData getPOIByAprilTag(int aprilTagId) {
        return aprilTagMap.get(aprilTagId);
    }
    
    /**
     * Gets POI data by ID.
     * 
     * @param poiId POI ID to look up
     * @return POI data, or null if not found
     */
    public POIData getPOIById(String poiId) {
        return poiMap.get(poiId);
    }
    
    /**
     * Gets all POIs with AprilTag mappings.
     * 
     * @return Map of AprilTag ID to POI data
     */
    public Map<Integer, POIData> getAllAprilTagPOIs() {
        return new HashMap<>(aprilTagMap);
    }
    
    /**
     * Checks if an AprilTag ID is valid.
     * 
     * @param aprilTagId AprilTag ID to check
     * @return True if valid, false otherwise
     */
    public boolean isValidAprilTag(int aprilTagId) {
        return aprilTagMap.containsKey(aprilTagId);
    }
    
    /**
     * Gets the absolute position of a POI.
     * 
     * @param aprilTagId AprilTag ID of the POI
     * @return Absolute position, or null if not found
     */
    public Pose2d getAbsolutePosition(int aprilTagId) {
        POIData poiData = getPOIByAprilTag(aprilTagId);
        return poiData != null ? poiData.position : null;
    }
}
