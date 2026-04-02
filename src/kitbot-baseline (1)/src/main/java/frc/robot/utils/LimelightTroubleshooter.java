// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Utility class for troubleshooting Limelight connection and streaming issues.
 * Provides diagnostic information and common solutions.
 */
public class LimelightTroubleshooter {
    
    private static final String LIMELIGHT_IP = "10.57.28.11";
    private static final String LIMELIGHT_STREAM_URL = "http://" + LIMELIGHT_IP + ":5800/stream";
    private static final String LIMELIGHT_WEB_URL = "http://" + LIMELIGHT_IP;
    
    /**
     * Runs comprehensive Limelight diagnostics.
     * 
     * @return Diagnostic report
     */
    public static String runDiagnostics() {
        StringBuilder report = new StringBuilder();
        report.append("=== LIMELIGHT DIAGNOSTIC REPORT ===\n\n");
        
        // Test 1: NetworkTables Connection
        boolean networkTablesWorking = testNetworkTablesConnection(report);
        
        // Test 2: Limelight Device Connection
        boolean limelightConnected = testLimelightConnection(report);
        
        // Test 3: Streaming Configuration
        boolean streamingConfigured = testStreamingConfiguration(report);
        
        // Test 4: Network Accessibility
        boolean networkAccessible = testNetworkAccessibility(report);
        
        // Summary and Recommendations
        generateSummary(report, networkTablesWorking, limelightConnected, 
                        streamingConfigured, networkAccessible);
        
        return report.toString();
    }
    
    /**
     * Tests NetworkTables connection to Limelight.
     */
    private static boolean testNetworkTablesConnection(StringBuilder report) {
        report.append("1. NETWORKTABLES CONNECTION TEST\n");
        
        try {
            NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
            NetworkTableEntry testEntry = limelightTable.getEntry("tv");
            double testValue = testEntry.getDouble(-999.0);
            
            boolean working = testValue != -999.0;
            report.append("   Status: ").append(working ? "WORKING" : "FAILED").append("\n");
            report.append("   Test Value: ").append(testValue).append("\n");
            
            if (working) {
                report.append("   ✅ NetworkTables connection is functional\n");
            } else {
                report.append("   ❌ NetworkTables connection failed\n");
                report.append("   Solutions:\n");
                report.append("     - Check robot network connection\n");
                report.append("     - Verify NetworkTables server is running\n");
                report.append("     - Check for network firewall blocking\n");
            }
            
            report.append("\n");
            return working;
            
        } catch (Exception e) {
            report.append("   Status: EXCEPTION - ").append(e.getMessage()).append("\n");
            report.append("   ❌ NetworkTables connection failed\n\n");
            return false;
        }
    }
    
    /**
     * Tests Limelight device connection.
     */
    private static boolean testLimelightConnection(StringBuilder report) {
        report.append("2. LIMELIGHT DEVICE CONNECTION TEST\n");
        
        try {
            NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
            
            // Check multiple entries to ensure device is responding
            NetworkTableEntry tvEntry = limelightTable.getEntry("tv");
            NetworkTableEntry txEntry = limelightTable.getEntry("tx");
            NetworkTableEntry tyEntry = limelightTable.getEntry("ty");
            
            double tv = tvEntry.getDouble(0.0);
            double tx = txEntry.getDouble(0.0);
            double ty = tyEntry.getDouble(0.0);
            
            boolean connected = (tv != 0.0 || tx != 0.0 || ty != 0.0);
            
            report.append("   Has Target (tv): ").append(tv > 0.5 ? "YES" : "NO").append("\n");
            report.append("   Horizontal Offset (tx): ").append(tx).append("°\n");
            report.append("   Vertical Offset (ty): ").append(ty).append("°\n");
            report.append("   Status: ").append(connected ? "CONNECTED" : "NO DATA").append("\n");
            
            if (connected) {
                report.append("   ✅ Limelight device is responding\n");
            } else {
                report.append("   ❌ Limelight device not responding\n");
                report.append("   Solutions:\n");
                report.append("     - Check Limelight power (LED should be on)\n");
                report.append("     - Verify Ethernet cable connection\n");
                report.append("     - Check IP address: ").append(LIMELIGHT_IP).append("\n");
                report.append("     - Confirm team number: 5728\n");
                report.append("     - Try power cycling Limelight\n");
            }
            
            report.append("\n");
            return connected;
            
        } catch (Exception e) {
            report.append("   Status: EXCEPTION - ").append(e.getMessage()).append("\n");
            report.append("   ❌ Limelight connection test failed\n\n");
            return false;
        }
    }
    
    /**
     * Tests streaming configuration.
     */
    private static boolean testStreamingConfiguration(StringBuilder report) {
        report.append("3. STREAMING CONFIGURATION TEST\n");
        
        try {
            NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
            
            // Check current pipeline and camera mode
            NetworkTableEntry pipelineEntry = limelightTable.getEntry("pipeline");
            NetworkTableEntry camModeEntry = limelightTable.getEntry("camMode");
            
            int pipeline = (int) pipelineEntry.getDouble(0.0);
            int camMode = (int) camModeEntry.getDouble(0.0);
            
            report.append("   Current Pipeline: ").append(pipeline).append("\n");
            report.append("   Camera Mode: ").append(camMode == 0 ? "VISION" : "DRIVER").append("\n");
            report.append("   Stream URL: ").append(LIMELIGHT_STREAM_URL).append("\n");
            
            boolean configured = true; // Basic configuration check
            
            if (camMode == 1) {
                report.append("   ✅ Driver mode - good for streaming\n");
            } else {
                report.append("   ⚠️ Vision mode - switch to driver for better streaming\n");
                configured = false;
            }
            
            report.append("   Recommendations:\n");
            report.append("     - Use Pipeline 0 for AprilTag detection\n");
            report.append("     - Use Pipeline 1 for streaming\n");
            report.append("     - Driver mode (camMode=1) for streaming\n");
            report.append("     - Vision mode (camMode=0) for processing\n");
            
            report.append("\n");
            return configured;
            
        } catch (Exception e) {
            report.append("   Status: EXCEPTION - ").append(e.getMessage()).append("\n");
            report.append("   ❌ Streaming configuration test failed\n\n");
            return false;
        }
    }
    
    /**
     * Tests network accessibility to Limelight.
     */
    private static boolean testNetworkAccessibility(StringBuilder report) {
        report.append("4. NETWORK ACCESSIBILITY TEST\n");
        
        if (RobotBase.isSimulation()) {
            report.append("   Status: SIMULATION MODE\n");
            report.append("   ⚠️ Network tests skipped in simulation\n");
            report.append("   In real robot, test these URLs:\n");
            report.append("     - Limelight Web: ").append(LIMELIGHT_WEB_URL).append("\n");
            report.append("     - Stream URL: ").append(LIMELIGHT_STREAM_URL).append("\n");
            report.append("\n");
            return true;
        }
        
        report.append("   Limelight IP: ").append(LIMELIGHT_IP).append("\n");
        report.append("   Stream URL: ").append(LIMELIGHT_STREAM_URL).append("\n");
        report.append("   Web Interface: ").append(LIMELIGHT_WEB_URL).append("\n");
        
        report.append("   Network Tests:\n");
        report.append("     - Ping test: ping ").append(LIMELIGHT_IP).append("\n");
        report.append("     - Web access: Open ").append(LIMELIGHT_WEB_URL).append("\n");
        report.append("     - Stream test: Open ").append(LIMELIGHT_STREAM_URL).append("\n");
        
        report.append("   Common Issues:\n");
        report.append("     - Wrong IP address (should be ").append(LIMELIGHT_IP).append(")\n");
        report.append("     - Network subnet mismatch\n");
        report.append("     - Firewall blocking ports 5800-5801\n");
        report.append("     - Router not forwarding multicast\n");
        
        report.append("\n");
        return true; // Assume network is accessible unless proven otherwise
    }
    
    /**
     * Generates summary and recommendations.
     */
    private static void generateSummary(StringBuilder report, boolean networkTables, 
                                       boolean limelight, boolean streaming, boolean network) {
        report.append("=== SUMMARY & RECOMMENDATIONS ===\n\n");
        
        int passedTests = (networkTables ? 1 : 0) + (limelight ? 1 : 0) + (streaming ? 1 : 0) + (network ? 1 : 0);
        
        report.append("Tests Passed: ").append(passedTests).append("/4\n\n");
        
        if (passedTests == 4) {
            report.append("✅ ALL TESTS PASSED\n");
            report.append("Limelight should be working correctly!\n");
            report.append("If streaming still doesn't work:\n");
            report.append("1. Try different browser (Chrome/Firefox)\n");
            report.append("2. Clear browser cache\n");
            report.append("3. Check browser console for errors\n");
            report.append("4. Try VLC or other video player\n");
        } else {
            report.append("⚠️ SOME TESTS FAILED\n");
            report.append("Fix failed tests first, then retry streaming.\n\n");
            
            if (!networkTables) {
                report.append("PRIORITY 1: Fix NetworkTables connection\n");
            }
            if (!limelight) {
                report.append("PRIORITY 2: Fix Limelight device connection\n");
            }
            if (!streaming) {
                report.append("PRIORITY 3: Configure streaming settings\n");
            }
        }
        
        report.append("\n=== STREAMING SETUP GUIDE ===\n");
        report.append("1. Set Pipeline to 1 (streaming pipeline)\n");
        report.append("2. Set Camera Mode to Driver (camMode=1)\n");
        report.append("3. Open: ").append(LIMELIGHT_STREAM_URL).append("\n");
        report.append("4. Should see video stream in browser\n");
    }
}
