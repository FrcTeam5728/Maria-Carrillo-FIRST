package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.CameraFeedStreamer;

/**
 * Command for controlling single camera streaming.
 * Simple on/off control for one camera.
 */
public class CameraControlCommand extends Command {
    
    private final CameraFeedStreamer streamer;
    private final boolean enableStreaming;
    
    /**
     * Creates a new CameraControlCommand.
     * 
     * @param streamer The camera feed streamer
     * @param enableStreaming Whether to enable or disable streaming
     */
    public CameraControlCommand(CameraFeedStreamer streamer, boolean enableStreaming) {
        this.streamer = streamer;
        this.enableStreaming = enableStreaming;
        
        addRequirements(); // No subsystem requirements
    }
    
    @Override
    public void initialize() {
        if (streamer == null) {
            System.err.println("CameraFeedStreamer is null - cannot control camera");
            return;
        }
        
        if (enableStreaming) {
            System.out.println("Enabling camera streaming");
            streamer.startStreaming();
        } else {
            System.out.println("Disabling camera streaming");
            streamer.stopStreaming();
        }
    }
    
    @Override
    public boolean isFinished() {
        return true; // One-shot command
    }
}
