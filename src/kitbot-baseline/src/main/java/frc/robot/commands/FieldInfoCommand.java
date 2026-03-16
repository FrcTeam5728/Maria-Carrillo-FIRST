package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.FieldLayoutChecker;

/**
 * Command that displays field layout and shooting target information.
 * Useful for debugging and understanding available targets.
 */
public class FieldInfoCommand extends CommandBase {
    
    private boolean hasRun = false;
    
    @Override
    public void initialize() {
        System.out.println("=== FIELD AND SHOOTING TARGET INFO ===");
        
        // Print field layout information
        FieldLayoutChecker.printFieldInfo();
        
        // Print shooting target information
        System.out.println("\n=== SHOOTING TARGETS ===");
        int[] shootingTargets = FieldLayoutChecker.getShootingTargetTags();
        
        for (int tagId : shootingTargets) {
            String info = FieldLayoutChecker.getShootingTargetInfo(tagId);
            boolean isTarget = FieldLayoutChecker.isShootingTarget(tagId);
            System.out.println(info + (isTarget ? " [SHOOTING TARGET]" : ""));
        }
        
        System.out.println("==========================\n");
        
        hasRun = true;
    }
    
    @Override
    public boolean isFinished() {
        return hasRun;
    }
}
