// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;

/**
 * Autonomous command that:
 * 1. Goes over the ramp
 * 2. Does a zigzag pattern in neutral zone to intake balls
 * 3. Goes over the other ramp
 * 4. Puts front against hopper
 * 5. Shoots all balls
 */
public class RampZigzagShootAuto extends Command {
    private final DriveSubsystem driveSubsystem;
    private final FuelSubsystem fuelSubsystem;
    
    private final Timer timer = new Timer();
    private int currentStep = 0;
    
    // Timing constants (in seconds) - based on field schematics
    private static final double OVER_RAMP_TIME = 3.0;        // Time to cross ~8ft ramp
    private static final double ZIGZAG_DISTANCE = 2.0;      // 2ft zigzag segments in neutral zone
    private static final double ZIGZAG_TIME = 1.2;          // 1.2s per zigzag segment
    private static final double TO_HOPPER_TIME = 2.0;          // Drive to hopper position
    private static final double SHOOT_TIME = 4.0;             // Extended shooting time
    
    // Speed constants - tuned for precise movement
    private static final double DRIVE_SPEED = 0.6;           // Moderate forward speed
    private static final double TURN_SPEED = 0.5;            // Zigzag turn intensity
    
    public RampZigzagShootAuto(DriveSubsystem drive, FuelSubsystem fuel) {
        driveSubsystem = drive;
        fuelSubsystem = fuel;
        addRequirements(drive, fuel);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        currentStep = 0;
        System.out.println("Starting Ramp-Zigzag-Shoot Autonomous");
    }

    @Override
    public void execute() {
        double currentTime = timer.get();
        
        switch (currentStep) {
            case 0:
                // Step 1: Go over the ramp
                if (currentTime < OVER_RAMP_TIME) {
                    driveSubsystem.driveArcade(() -> DRIVE_SPEED, () -> 0);
                    System.out.println("Step 1: Going over ramp (" + currentTime + "s)");
                } else {
                    currentStep = 1;
                    timer.reset();
                }
                break;
                
            case 1:
                // Step 2: Zigzag in neutral zone to intake balls
                if (currentTime < ZIGZAG_TIME * 4) { // 4 zigzag segments
                    double segmentTime = currentTime % ZIGZAG_TIME;
                    if (segmentTime < ZIGZAG_TIME / 2) {
                        // Zigzag left
                        driveSubsystem.driveArcade(() -> DRIVE_SPEED, () -> TURN_SPEED);
                    } else {
                        // Zigzag right
                        driveSubsystem.driveArcade(() -> DRIVE_SPEED, () -> -TURN_SPEED);
                    }
                    fuelSubsystem.intake(); // Intake while zigzagging
                    System.out.println("Step 2: Zigzagging to intake balls (" + currentTime + "s)");
                } else {
                    currentStep = 2;
                    timer.reset();
                }
                break;
                
            case 2:
                // Step 3: Go over the other ramp
                if (currentTime < OVER_RAMP_TIME) {
                    driveSubsystem.driveArcade(() -> DRIVE_SPEED, () -> 0);
                    fuelSubsystem.stop(); // Stop intake
                    System.out.println("Step 3: Going over other ramp (" + currentTime + "s)");
                } else {
                    currentStep = 3;
                    timer.reset();
                }
                break;
                
            case 3:
                // Step 4: Put front against hopper
                if (currentTime < TO_HOPPER_TIME) {
                    driveSubsystem.driveArcade(() -> -DRIVE_SPEED * 0.5, () -> 0); // Slow reverse to position against hopper
                    System.out.println("Step 4: Positioning against hopper (" + currentTime + "s)");
                } else {
                    currentStep = 4;
                    timer.reset();
                }
                break;
                
            case 4:
                // Step 5: Shoot all balls
                if (currentTime < SHOOT_TIME) {
                    driveSubsystem.driveArcade(() -> 0, () -> 0); // Stop driving
                    fuelSubsystem.launch(); // Shoot balls
                    System.out.println("Step 5: Shooting balls (" + currentTime + "s)");
                } else {
                    currentStep = 5;
                }
                break;
                
            default:
                // Autonomous complete
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveArcade(() -> 0, () -> 0); // Stop driving using driveArcade
        fuelSubsystem.stop();
        System.out.println("Ramp-Zigzag-Shoot Autonomous " + (interrupted ? "interrupted" : "completed"));
    }

    @Override
    public boolean isFinished() {
        return currentStep >= 5; // Complete after all 5 steps
    }
}
