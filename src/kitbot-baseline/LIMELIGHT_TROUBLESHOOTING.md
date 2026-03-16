# Limelight USB Connection Troubleshooting

## Problem
Robot does not recognize that the Limelight is connected via USB.

## Diagnostic Tools Added

### 1. Automatic Connection Testing
- **Startup Test**: Runs automatically when robot starts
- **Periodic Check**: Every 5 seconds during operation
- **Console Output**: Shows connection status and error messages

### 2. Manual Diagnostic Command
- **Button**: Driver Controller **BACK** button
- **Function**: Runs comprehensive Limelight diagnostic
- **Output**: Detailed connection status and troubleshooting tips

## What to Check First

### Physical Connection
1. **Power**: Limelight LED should be illuminated (usually green/blue)
2. **USB Cable**: 
   - Firmly connected to both Limelight and roboRIO
   - Use high-quality, short cable (avoid long cables or extensions)
   - Check for bent pins or damage
3. **USB Port**: Try different USB ports on roboRIO if available

### Software Configuration
1. **Limelight Name**: Must be configured as "limelight" in web interface
2. **NetworkTables**: Must be working properly
3. **Firmware**: Ensure Limelight firmware is up to date

## Debug Output Messages

### Normal Operation
```
=== LIMELIGHT CONNECTION TEST ===
Table path: limelight
tv (valid target): 0.0
tx (horizontal offset): 0.0
ty (vertical offset): 0.0
tid (target ID): -1.0
Limelight connection appears to be working
=================================
```

### Connection Problems
```
=== LIMELIGHT CONNECTION TEST ===
Table path: limelight
tv (valid target): -1.0
tx (horizontal offset): 999.0
ty (vertical offset): 999.0
tid (target ID): -1.0
WARNING: Limelight may not be connected or configured!
Check:
1. Limelight is powered on (LED should be on)
2. USB cable is securely connected
3. Limelight is configured with name 'limelight'
4. NetworkTables are working
=================================
```

## Step-by-Step Troubleshooting

### Step 1: Physical Check
- [ ] Limelight power LED is on
- [ ] USB cable securely connected at both ends
- [ ] No visible damage to cable or connectors
- [ ] Try different USB cable if available

### Step 2: Power Cycle
1. Turn off robot power
2. Disconnect USB cable
3. Wait 10 seconds
4. Reconnect USB cable
5. Power on robot
6. Check console output

### Step 3: Software Check
- [ ] Access Limelight web interface (usually at 10.te.am.xx.xx:5801)
- [ ] Verify Limelight name is set to "limelight"
- [ ] Check that NetworkTables are enabled
- [ ] Verify pipeline 0 is selected for AprilTag detection

### Step 4: USB Port Test
- [ ] Try different USB port on roboRIO
- [ ] Avoid using USB hubs
- [ ] Connect directly to roboRIO

### Step 5: Advanced Troubleshooting
- [ ] Check roboRIO USB port functionality with other devices
- [ ] Verify Limelight firmware version
- [ ] Check for IP conflicts on network
- [ ] Test with known-good Limelight if available

## Common Issues and Solutions

### Issue: "Limelight not responding - check connection"
**Cause**: USB connection problem or Limelight not powered
**Solution**: Check physical connections and power

### Issue: All values show as defaults (-1, 999)
**Cause**: NetworkTables not communicating with Limelight
**Solution**: Verify Limelight configuration and network setup

### Issue: Intermittent connection
**Cause**: Loose USB connection or cable quality
**Solution**: Use better quality cable, secure connections

## Testing the Fix

1. Deploy updated code to robot
2. Power on robot
3. Watch console output for connection test results
4. Press **BACK** button on driver controller for detailed diagnostic
5. Follow troubleshooting steps based on output

## Expected Behavior When Working

- Robot startup shows "Limelight connection appears to be working"
- When AprilTag is in view: `hasTarget()` returns true
- Target data updates continuously in `periodic()`
- No error messages about connection

## Technical Details

The code now includes:
- Connection testing on startup
- Periodic connection monitoring
- Comprehensive diagnostic command
- Detailed error reporting
- Troubleshooting guidance

Use these tools to identify and resolve the USB connection issue.
