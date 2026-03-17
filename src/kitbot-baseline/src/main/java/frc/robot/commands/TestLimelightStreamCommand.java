// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.net.HttpURLConnection;
import java.net.URL;
import java.io.IOException;

/**
 * Command to test direct Limelight stream access.
 * This bypasses CameraServer and tests the raw MJPEG stream.
 */
public class TestLimelightStreamCommand extends CommandBase {
    
    private static final String LIMELIGHT_IP = "172.22.11.2";
    private static final String STREAM_URL = "http://" + LIMELIGHT_IP + ":5800/stream.mjpg";
    private static final String WEB_URL = "http://" + LIMELIGHT_IP + ":5801";
    
    private boolean testComplete = false;
    private boolean streamWorking = false;
    private boolean webWorking = false;
    
    public TestLimelightStreamCommand() {}
    
    @Override
    public void initialize() {
        System.out.println("=== LIMELIGHT STREAM TEST ===");
        System.out.println("Testing direct stream access...");
        System.out.println("Stream URL: " + STREAM_URL);
        System.out.println("Web Interface: " + WEB_URL);
        System.out.println("================================");
        
        testComplete = false;
        streamWorking = false;
        webWorking = false;
        
        // Run tests in background to avoid blocking
        new Thread(this::performTests).start();
    }
    
    @Override
    public void execute() {
        // Nothing to do - tests run in background
    }
    
    @Override
    public void end(boolean interrupted) {
        if (!testComplete) {
            System.out.println("Stream test interrupted.");
        }
    }
    
    @Override
    public boolean isFinished() {
        return testComplete;
    }
    
    /**
     * Performs the actual stream tests.
     */
    private void performTests() {
        try {
            // Test 1: Check if stream URL responds
            System.out.println("Testing MJPEG stream...");
            streamWorking = testUrlConnection(STREAM_URL, 5000);
            
            // Test 2: Check if web interface responds
            System.out.println("Testing web interface...");
            webWorking = testUrlConnection(WEB_URL, 3000);
            
            // Results
            System.out.println("\n=== TEST RESULTS ===");
            System.out.println("MJPEG Stream: " + (streamWorking ? "✅ WORKING" : "❌ FAILED"));
            System.out.println("Web Interface: " + (webWorking ? "✅ WORKING" : "❌ FAILED"));
            
            if (streamWorking) {
                System.out.println("\n🎥 STREAM IS ACCESSIBLE!");
                System.out.println("Add to Shuffleboard:");
                System.out.println("1. Add Camera Server widget");
                System.out.println("2. Enter URL: " + STREAM_URL);
                System.out.println("3. Name: Limelight");
            } else {
                System.out.println("\n❌ STREAM NOT ACCESSIBLE");
                System.out.println("Troubleshooting:");
                System.out.println("1. Check Limelight USB connection to roboRIO");
                System.out.println("2. Verify streaming is enabled in Limelight");
                System.out.println("3. Access web interface: " + WEB_URL);
                System.out.println("4. Check 'Stream' checkbox in Limelight settings");
            }
            
            // Update SmartDashboard
            SmartDashboard.putBoolean("LimelightStreamTest/StreamWorking", streamWorking);
            SmartDashboard.putBoolean("LimelightStreamTest/WebWorking", webWorking);
            SmartDashboard.putString("LimelightStreamTest/StreamURL", STREAM_URL);
            SmartDashboard.putString("LimelightStreamTest/WebURL", WEB_URL);
            SmartDashboard.putString("LimelightStreamTest/Status", 
                streamWorking ? "STREAM WORKING" : "STREAM FAILED");
            
        } catch (Exception e) {
            System.err.println("Stream test error: " + e.getMessage());
            SmartDashboard.putString("LimelightStreamTest/Error", e.getMessage());
        }
        
        testComplete = true;
    }
    
    /**
     * Tests if a URL responds with HTTP OK.
     * 
     * @param url The URL to test
     * @param timeout Timeout in milliseconds
     * @return True if URL responds with 200 OK
     */
    private boolean testUrlConnection(String url, int timeout) {
        try {
            HttpURLConnection connection = (HttpURLConnection) new URL(url).openConnection();
            connection.setRequestMethod("HEAD");
            connection.setConnectTimeout(timeout);
            connection.setReadTimeout(timeout);
            
            int responseCode = connection.getResponseCode();
            return responseCode == 200; // HTTP OK
            
        } catch (IOException e) {
            return false;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Gets the stream URL for manual testing.
     * 
     * @return The MJPEG stream URL
     */
    public static String getStreamUrl() {
        return STREAM_URL;
    }
    
    /**
     * Gets the web interface URL.
     * 
     * @return The web interface URL
     */
    public static String getWebUrl() {
        return WEB_URL;
    }
}
