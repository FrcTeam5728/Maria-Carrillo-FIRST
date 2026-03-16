# Limelight Connection Troubleshooting

## Quick Test Commands

**Driver Controller:**
- **BACK Button**: Run continuous Limelight diagnostic
- **START Button**: Quick connection test

**Operator Controller:**
- **B Button**: Auto-shoot (requires working Limelight)
- **A Button**: Toggle pulsing shooter
- **Y Button**: Manual continuous shooting

## Step-by-Step Troubleshooting

### 1. Basic Connection Test
1. Press **START button** on driver controller
2. Check console output for connection status
3. Look for "✅ Limelight is connected and responding!"

### 2. Continuous Monitoring
1. Press **BACK button** on driver controller
2. Watch real-time Limelight status
3. Point Limelight at AprilTag to test target detection

### 3. Physical Connection Checks

**Power:**
- Limelight LED should be illuminated (green/blue)
- Check power cable connection
- Verify 12V power supply

**Network Connection:**
- USB: Check USB cable to roboRIO
- Ethernet: Verify Ethernet cable connection
- Check cable integrity

**IP Address:**
- Default: 10.TE.AM.XX (where TE=Team, AM=Robot, XX=limelight)
- Example: 10.57.28.11 for team 5728
- Access via web browser for configuration

### 4. NetworkTables Debugging

**SmartDashboard Values:**
- `Limelight/Connected`: Should be true
- `Limelight/HasTarget`: True when AprilTag visible
- `Limelight/TargetID`: Should show AprilTag number
- `Limelight/Distance`: Calculated distance in meters

**NetworkTables Entries:**
- `limelight/tv`: Target valid (0/1)
- `limelight/tx`: Horizontal offset (-27 to 27)
- `limelight/ty`: Vertical offset (-20.5 to 20.5)
- `limelight/ta`: Target area (0 to 100)
- `limelight/tid`: Target ID

### 5. Common Issues & Solutions

**❌ "Limelight not connected"**
- Check physical cable connections
- Verify Limelight power (LED on)
- Restart NetworkTables (reboot roboRIO)
- Check IP address conflicts

**❌ "No target detected"**
- Point Limelight directly at AprilTag
- Check lighting conditions
- Verify correct pipeline selected
- AprilTag may be too far (>10m)

**❌ "Target ID: -1"**
- AprilTag not in view
- Wrong pipeline (need AprilTag pipeline)
- Camera obstruction

**❌ A/B/Y buttons don't work**
- Limelight must be connected first
- Press START button to test connection
- Check SmartDashboard for Limelight status

### 6. Manual Limelight Setup

**Web Interface:**
1. Open browser to Limelight IP address
2. Check "Connection" tab for network status
3. Verify "Input" tab shows camera feed
4. Set "Pipeline" to AprilTag mode (usually pipeline 0)

**Pipeline Settings:**
- Pipeline 0: AprilTag detection
- Pipeline 1-9: Custom vision modes
- LED Mode: 3 (on) for testing

### 7. Advanced Troubleshooting

**NetworkTables Issues:**
```bash
# Check NetworkTables connection
ntcore -server 10.TE.AM.XX -list
```

**Firewall:**
- Port 5810 for TCP
- Port 5800-5810 for UDP
- Disable Windows Firewall if needed

**USB Issues:**
- Try different USB port on roboRIO
- Check USB cable quality
- Verify USB 2.0 compatibility

## Expected Behavior

**Working System:**
1. Press START → "✅ Limelight is connected and responding!"
2. Point at AprilTag → "✅ Target detected!"
3. Press B → Auto-shooting activates
4. A button toggles pulsing shooter
5. Y button for manual shooting

**Broken System:**
1. Press START → "❌ Limelight not connected"
2. A/B/Y buttons won't work for shooting
3. Need to fix connection first

## Quick Fix Checklist

- [ ] Limelight LED is on
- [ ] Cable connections secure
- [ ] IP address accessible via browser
- [ ] NetworkTables values updating
- [ ] AprilTag visible in camera feed
- [ ] Correct pipeline selected
- [ ] SmartDashboard shows connection

If all items checked and still not working, the issue may be:
- Hardware failure (Limelight malfunction)
- Network configuration problem
- Software version incompatibility
