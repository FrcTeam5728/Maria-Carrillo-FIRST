// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

/**
 * Command to find Limelight on the network by scanning common IPs.
 * This helps identify the correct IP address when the Limelight is unreachable.
 */
public class FindLimelightCommand extends CommandBase {
    
    // Common Limelight IP ranges to scan
    private static final String[] COMMON_LIMELIGHT_IPS = {
        "10.57.28.11",  // Your current configured IP
        "10.57.28.1",   // Team 5728 common variations
        "10.57.28.2",
        "10.57.28.10",
        "10.57.28.20",
        "192.168.1.90", // Home network common
        "192.168.0.90",
        "172.22.11.2",  // Practice robot common
        "localhost",     // Local testing
        "127.0.0.1"
    };
    
    private boolean scanComplete = false;
    private String foundIp = null;
    
    public FindLimelightCommand() {}
    
    @Override
    public void initialize() {
        System.out.println("=== LIMELIGHT NETWORK SCANNER ===");
        System.out.println("Scanning for Limelight devices...");
        System.out.println("This may take up to 30 seconds...\n");
        
        scanComplete = false;
        foundIp = null;
        
        // Run scan in background to avoid blocking
        CompletableFuture.runAsync(this::performNetworkScan);
    }
    
    @Override
    public void execute() {
        // Nothing to do during execution - scan runs in background
    }
    
    @Override
    public void end(boolean interrupted) {
        if (!scanComplete) {
            System.out.println("\nScan interrupted.");
        }
    }
    
    @Override
    public boolean isFinished() {
        return scanComplete;
    }
    
    /**
     * Performs the actual network scan.
     */
    private void performNetworkScan() {
        boolean found = false;
        
        for (String ip : COMMON_LIMELIGHT_IPS) {
            System.out.print("Testing " + ip + "... ");
            
            if (testLimelightConnection(ip)) {
                System.out.println("✅ FOUND LIMELIGHT!");
                foundIp = ip;
                found = true;
                
                System.out.println("\n=== LIMELIGHT FOUND ===");
                System.out.println("IP Address: " + ip);
                System.out.println("Stream URL: http://" + ip + ":5800/stream.mjpg");
                System.out.println("Web Interface: http://" + ip + ":5801");
                System.out.println("\nTo fix your connection:");
                System.out.println("1. Update LIMELIGHT_IP constants to: " + ip);
                System.out.println("2. Rebuild and redeploy your code");
                System.out.println("3. Test the camera stream in Shuffleboard");
                break;
            } else {
                System.out.println("❌ No response");
            }
        }
        
        if (!found) {
            System.out.println("\n=== NO LIMELIGHT FOUND ===");
            System.out.println("Suggestions:");
            System.out.println("1. Check if Limelight is powered on");
            System.out.println("2. Verify network cable connection");
            System.out.println("3. Check if you're on the correct network (robot radio)");
            System.out.println("4. Try accessing Limelight web interface directly");
            System.out.println("5. Consider factory resetting the Limelight");
        }
        
        scanComplete = true;
    }
    
    /**
     * Tests if a Limelight is responding at the given IP.
     * 
     * @param ip The IP address to test
     * @return True if Limelight responds
     */
    private boolean testLimelightConnection(String ip) {
        try {
            // Test basic network connectivity
            InetAddress address = InetAddress.getByName(ip);
            boolean reachable = address.isReachable(2000); // 2 second timeout
            
            if (reachable) {
                // Test Limelight-specific port (5800 for stream)
                return testPort(ip, 5800, 1000);
            }
            
            return false;
            
        } catch (UnknownHostException e) {
            return false;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Tests if a specific port is open on the given IP.
     * 
     * @param ip The IP address
     * @param port The port to test
     * @param timeout Timeout in milliseconds
     * @return True if port is open
     */
    private boolean testPort(String ip, int port, int timeout) {
        try (java.net.Socket socket = new java.net.Socket()) {
            socket.connect(new java.net.InetSocketAddress(ip, port), timeout);
            return true;
        } catch (Exception e) {
            return false;
        }
    }
    
    /**
     * Gets the IP address of the found Limelight.
     * 
     * @return The IP address, or null if not found
     */
    public String getFoundIp() {
        return foundIp;
    }
}
