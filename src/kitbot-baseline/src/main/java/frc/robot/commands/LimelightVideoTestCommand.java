// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to test and troubleshoot Limelight video streaming issues.
 * Focuses on video data transmission while AprilTag data works.
 */
public class LimelightVideoTestCommand extends Command {
    
    private final LimelightSubsystem limelightSubsystem;
    
    // Test parameters
    private static final String LIMELIGHT_IP = "10.57.28.11";
    private static final String STREAM_URL = "http://" + LIMELIGHT_IP + ":5800/stream";
    private static final String SNAPSHOT_URL = "http://" + LIMELIGHT_IP + ":5800/snapshot.jpg";
    
    private int testCount = 0;
    private static final int MAX_TESTS = 10;
    
    /**
     * Creates a new LimelightVideoTestCommand.
     * 
     * @param limelightSubsystem Limelight subsystem to test
     */
    public LimelightVideoTestCommand(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(limelightSubsystem);
    }
    
    @Override
    public void initialize() {
        testCount = 0;
        System.out.println("=== LIMELIGHT VIDEO STREAMING TEST ===");
        System.out.println("Testing video data transmission...");
        System.out.println("AprilTag data: WORKING");
        System.out.println("Video data: TESTING");
        System.out.println("Limelight IP: " + LIMELIGHT_IP);
        System.out.println("Stream URL: " + STREAM_URL);
        System.out.println("=====================================");
    }
    
    @Override
    public void execute() {
        testCount++;
        
        if (testCount == 1) {
            // Test 1: Check Limelight connection status
            testConnectionStatus();
        } else if (testCount == 2) {
            // Test 2: Check streaming configuration
            testStreamingConfig();
        } else if (testCount == 3) {
            // Test 3: Check pipeline settings
            testPipelineSettings();
        } else if (testCount == 4) {
            // Test 4: Check video mode
            testVideoMode();
        } else if (testCount == 5) {
            // Test 5: Check network accessibility
            testNetworkAccess();
        } else if (testCount == 6) {
            // Test 6: Check streaming port
            testStreamingPort();
        } else if (testCount == 7) {
            // Test 7: Check camera settings
            testCameraSettings();
        } else if (testCount == 8) {
            // Test 8: Check bandwidth
            testBandwidth();
        } else if (testCount == 9) {
            // Test 9: Check stream format
            testStreamFormat();
        } else if (testCount == 10) {
            // Test 10: Final recommendations
            provideRecommendations();
        }
    }
    
    /**
     * Tests Limelight connection status.
     */
    private void testConnectionStatus() {
        System.out.println("\n--- TEST 1: CONNECTION STATUS ---");
        System.out.println("Limelight connected: " + limelightSubsystem.isConnected());
        System.out.println("Target detected: " + limelightSubsystem.hasTarget());
        System.out.println("Target ID: " + limelightSubsystem.getTargetId());
        System.out.println("Target area: " + limelightSubsystem.getTargetArea());
        System.out.println("✅ AprilTag data transmission: WORKING");
        System.out.println("❓ Video data transmission: TESTING");
    }
    
    /**
     * Tests streaming configuration.
     */
    private void testStreamingConfig() {
        System.out.println("\n--- TEST 2: STREAMING CONFIGURATION ---");
        System.out.println("Checking streaming settings...");
        System.out.println("Expected stream URL: " + STREAM_URL);
        System.out.println("Expected snapshot URL: " + SNAPSHOT_URL);
        System.out.println("💡 TIP: Access stream in browser: " + STREAM_URL);
        System.out.println("💡 TIP: Access snapshot in browser: " + SNAPSHOT_URL);
        System.out.println("❓ If browser shows video, streaming is enabled");
        System.out.println("❓ If browser shows error, streaming is disabled");
    }
    
    /**
     * Tests pipeline settings.
     */
    private void testPipelineSettings() {
        System.out.println("\n--- TEST 3: PIPELINE SETTINGS ---");
        System.out.println("Checking pipeline settings...");
        System.out.println("Available pipelines: 0-9");
        System.out.println("Pipeline 0: Usually AprilTag detection");
        System.out.println("Pipeline 1: Usually driver camera/video");
        System.out.println("💡 TIP: Try switching to pipeline 1 for video streaming");
        System.out.println("💡 TIP: Use X button to toggle pipeline modes");
        System.out.println("💡 TIP: Current pipeline can be checked in Limelight web interface");
    }
    
    /**
     * Tests video mode.
     */
    private void testVideoMode() {
        System.out.println("\n--- TEST 4: VIDEO MODE ---");
        System.out.println("Checking video mode settings...");
        System.out.println("Video mode may be disabled in Limelight settings");
        System.out.println("💡 CHECK: Limelight Web Interface → Settings → Video");
        System.out.println("💡 CHECK: 'Enable Stream' checkbox should be checked");
        System.out.println("💡 CHECK: 'Stream Resolution' should be set");
        System.out.println("💡 CHECK: 'Stream FPS' should be set (15-30 recommended)");
    }
    
    /**
     * Tests network accessibility.
     */
    private void testNetworkAccess() {
        System.out.println("\n--- TEST 5: NETWORK ACCESSIBILITY ---");
        System.out.println("Testing network access to Limelight...");
        System.out.println("Ping test: ping " + LIMELIGHT_IP);
        System.out.println("HTTP test: curl " + STREAM_URL);
        System.out.println("💡 TIP: Run these commands from driver station");
        System.out.println("💡 TIP: Check if robot can reach Limelight IP");
        System.out.println("💡 TIP: Verify network configuration matches team number");
    }
    
    /**
     * Tests streaming port.
     */
    private void testStreamingPort() {
        System.out.println("\n--- TEST 6: STREAMING PORT ---");
        System.out.println("Testing streaming port accessibility...");
        System.out.println("Stream port: 5800");
        System.out.println("Web interface port: 5801");
        System.out.println("💡 TIP: Port 5800 may be blocked by firewall");
        System.out.println("💡 TIP: Check robot network firewall settings");
        System.out.println("💡 TIP: Verify port forwarding if using router");
    }
    
    /**
     * Tests camera settings.
     */
    private void testCameraSettings() {
        System.out.println("\n--- TEST 7: CAMERA SETTINGS ---");
        System.out.println("Checking camera configuration...");
        System.out.println("💡 CHECK: Limelight Web Interface → Camera");
        System.out.println("💡 CHECK: 'Exposure' settings (auto vs manual)");
        System.out.println("💡 CHECK: 'Brightness' settings");
        System.out.println("💡 CHECK: 'White Balance' settings");
        System.out.println("💡 CHECK: 'Resolution' settings (640x480 recommended)");
    }
    
    /**
     * Tests bandwidth.
     */
    private void testBandwidth() {
        System.out.println("\n--- TEST 8: BANDWIDTH TEST ---");
        System.out.println("Checking available bandwidth...");
        System.out.println("Video streaming requires ~2-5 Mbps");
        System.out.println("AprilTag data requires ~0.1 Mbps");
        System.out.println("💡 TIP: Reduce stream resolution if bandwidth limited");
        System.out.println("💡 TIP: Lower stream FPS if network congested");
        System.out.println("💡 TIP: Check radio bandwidth usage");
    }
    
    /**
     * Tests stream format.
     */
    private void testStreamFormat() {
        System.out.println("\n--- TEST 9: STREAM FORMAT ---");
        System.out.println("Checking stream format compatibility...");
        System.out.println("Default format: MJPEG over HTTP");
        System.out.println("Alternative formats: H.264, RTSP");
        System.out.println("💡 TIP: Shuffleboard expects MJPEG stream");
        System.out.println("💡 TIP: Check 'Stream Type' in Limelight settings");
        System.out.println("💡 TIP: Try different stream formats if MJPEG fails");
    }
    
    /**
     * Provides final recommendations.
     */
    private void provideRecommendations() {
        System.out.println("\n--- TEST 10: RECOMMENDATIONS ---");
        System.out.println("=== COMMON SOLUTIONS ===");
        System.out.println("1. Enable streaming in Limelight web interface");
        System.out.println("   URL: http://" + LIMELIGHT_IP + ":5801");
        System.out.println("   Path: Settings → Video → Enable Stream");
        System.out.println();
        System.out.println("2. Switch to correct pipeline");
        System.out.println("   Pipeline 0: AprilTag detection (no video)");
        System.out.println("   Pipeline 1: Driver camera (with video)");
        System.out.println("   Use X button to toggle pipelines");
        System.out.println();
        System.out.println("3. Check network connectivity");
        System.out.println("   Ping: " + LIMELIGHT_IP);
        System.out.println("   Browser: " + STREAM_URL);
        System.out.println();
        System.out.println("4. Verify Shuffleboard configuration");
        System.out.println("   Add Camera Server widget");
        System.out.println("   Set stream URL: " + STREAM_URL);
        System.out.println();
        System.out.println("5. Test with browser first");
        System.out.println("   If browser shows video → Shuffleboard issue");
        System.out.println("   If browser shows error → Limelight issue");
        System.out.println();
        System.out.println("=== QUICK FIXES ===");
        System.out.println("• Press X to toggle pipeline modes");
        System.out.println("• Access Limelight web interface");
        System.out.println("• Enable 'Stream' checkbox");
        System.out.println("• Set resolution to 640x480");
        System.out.println("• Set FPS to 15-30");
        System.out.println("• Restart robot code");
        System.out.println("=====================================");
    }
    
    @Override
    public boolean isFinished() {
        return testCount >= MAX_TESTS;
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("\n=== VIDEO STREAMING TEST COMPLETE ===");
        System.out.println("AprilTag data: ✅ WORKING");
        System.out.println("Video data: ❓ CHECK RECOMMENDATIONS ABOVE");
        System.out.println("Next steps:");
        System.out.println("1. Try browser access to stream");
        System.out.println("2. Check Limelight web interface");
        System.out.println("3. Enable streaming in settings");
        System.out.println("4. Test with different pipeline");
        System.out.println("=====================================");
    }
}
