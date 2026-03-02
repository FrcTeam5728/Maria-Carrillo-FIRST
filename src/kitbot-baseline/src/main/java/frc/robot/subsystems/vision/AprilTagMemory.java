// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Short-term memory system for AprilTag targets.
 * Maintains recent target data to provide smooth tracking even when tags are temporarily lost.
 */
public class AprilTagMemory {
    private static class TargetMemory {
        Pose2d lastKnownPose;
        double lastSeenTime;
        double lastDistance;
        double lastYaw;
        int consecutiveDetections;
        double confidence;
        
        TargetMemory(Pose2d pose, double distance, double yaw, double time) {
            this.lastKnownPose = pose;
            this.lastDistance = distance;
            this.lastYaw = yaw;
            this.lastSeenTime = time;
            this.consecutiveDetections = 1;
            this.confidence = 0.5; // Start with medium confidence
        }
        
        void update(Pose2d pose, double distance, double yaw, double time) {
            this.lastKnownPose = pose;
            this.lastDistance = distance;
            this.lastYaw = yaw;
            this.lastSeenTime = time;
            this.consecutiveDetections++;
            
            // Increase confidence with consecutive detections
            this.confidence = Math.min(1.0, this.confidence + 0.1);
        }
        
        void decay(double currentTime, double memoryDuration) {
            double timeSinceSeen = currentTime - lastSeenTime;
            if (timeSinceSeen > memoryDuration) {
                this.confidence = 0.0;
            } else {
                // Decay confidence based on time since last seen
                double decayFactor = 1.0 - (timeSinceSeen / memoryDuration);
                this.confidence *= decayFactor;
                this.consecutiveDetections = Math.max(0, this.consecutiveDetections - 1);
            }
        }
    }
    
    private final Map<Integer, TargetMemory> targetMemories;
    private final double memoryDurationSeconds;
    private final double maxDistanceForMemory;
    private final Timer timer;
    
    /**
     * Creates a new AprilTag memory system.
     * 
     * @param memoryDurationSeconds How long to remember targets after losing them
     * @param maxDistanceForMemory Maximum distance at which to remember targets
     */
    public AprilTagMemory(double memoryDurationSeconds, double maxDistanceForMemory) {
        this.targetMemories = new HashMap<>();
        this.memoryDurationSeconds = memoryDurationSeconds;
        this.maxDistanceForMemory = maxDistanceForMemory;
        this.timer = new Timer();
        this.timer.start();
    }
    
    /**
     * Updates the memory with new target data.
     * 
     * @param tagId The AprilTag ID
     * @param pose The detected pose of the tag
     * @param distance The distance to the tag
     * @param yaw The yaw angle to the tag
     */
    public void updateTarget(int tagId, Pose2d pose, double distance, double yaw) {
        double currentTime = timer.get();
        
        // Don't remember targets that are too far away
        if (distance > maxDistanceForMemory) {
            return;
        }
        
        TargetMemory memory = targetMemories.get(tagId);
        if (memory == null) {
            memory = new TargetMemory(pose, distance, yaw, currentTime);
            targetMemories.put(tagId, memory);
        } else {
            memory.update(pose, distance, yaw, currentTime);
        }
    }
    
    /**
     * Gets the best remembered target.
     * 
     * @return Optional containing the best target data (tagId, pose, distance, yaw, confidence)
     */
    public Optional<BestTarget> getBestRememberedTarget() {
        double currentTime = timer.get();
        
        // Decay all memories
        targetMemories.entrySet().removeIf(entry -> {
            entry.getValue().decay(currentTime, memoryDurationSeconds);
            return entry.getValue().confidence <= 0.0;
        });
        
        // Find the best target based on confidence and distance
        BestTarget best = null;
        for (Map.Entry<Integer, TargetMemory> entry : targetMemories.entrySet()) {
            TargetMemory memory = entry.getValue();
            
            if (memory.confidence > 0.0) {
                // Score based on confidence and recency
                double score = memory.confidence * (1.0 / (1.0 + memory.lastDistance));
                
                if (best == null || score > best.score) {
                    best = new BestTarget(
                        entry.getKey(),
                        memory.lastKnownPose,
                        memory.lastDistance,
                        memory.lastYaw,
                        memory.confidence,
                        score
                    );
                }
            }
        }
        
        return Optional.ofNullable(best);
    }
    
    /**
     * Gets a specific remembered target.
     * 
     * @param tagId The AprilTag ID
     * @return Optional containing the target data if remembered
     */
    public Optional<TargetData> getRememberedTarget(int tagId) {
        TargetMemory memory = targetMemories.get(tagId);
        if (memory != null && memory.confidence > 0.0) {
            return Optional.of(new TargetData(
                tagId,
                memory.lastKnownPose,
                memory.lastDistance,
                memory.lastYaw,
                memory.confidence
            ));
        }
        return Optional.empty();
    }
    
    /**
     * Clears all memory.
     */
    public void clearMemory() {
        targetMemories.clear();
    }
    
    /**
     * Gets the number of remembered targets.
     * 
     * @return Number of targets with confidence > 0
     */
    public int getRememberedTargetCount() {
        return (int) targetMemories.values().stream()
            .filter(memory -> memory.confidence > 0.0)
            .count();
    }
    
    /**
     * Represents the best remembered target.
     */
    public static class BestTarget {
        public final int tagId;
        public final Pose2d pose;
        public final double distance;
        public final double yaw;
        public final double confidence;
        public final double score; // Internal scoring for selection
        
        private BestTarget(int tagId, Pose2d pose, double distance, double yaw, double confidence, double score) {
            this.tagId = tagId;
            this.pose = pose;
            this.distance = distance;
            this.yaw = yaw;
            this.confidence = confidence;
            this.score = score;
        }
    }
    
    /**
     * Represents remembered target data.
     */
    public static class TargetData {
        public final int tagId;
        public final Pose2d pose;
        public final double distance;
        public final double yaw;
        public final double confidence;
        
        private TargetData(int tagId, Pose2d pose, double distance, double yaw, double confidence) {
            this.tagId = tagId;
            this.pose = pose;
            this.distance = distance;
            this.yaw = yaw;
            this.confidence = confidence;
        }
    }
}
