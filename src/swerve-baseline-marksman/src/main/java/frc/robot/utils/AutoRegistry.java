package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.CommandManager;
import frc.robot.subsystems.SubsystemManager;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;
import java.util.function.Consumer;

/**
 * Automatically discovers and registers all subsystems and commands in the robot code.
 * Uses reflection to find and instantiate classes at runtime.
 */
public class AutoRegistry {
    private static final String SUBSYSTEM_PACKAGE = "frc.robot.subsystems";
    private static final String COMMAND_PACKAGE = "frc.robot.commands";

    private final SubsystemManager subsystemManager;
    private final CommandManager commandManager;

    public AutoRegistry(SubsystemManager subsystemManager, CommandManager commandManager) {
        this.subsystemManager = subsystemManager;
        this.commandManager = commandManager;
    }

    /**
     * Discover and register all subsystems and commands
     */
    public void registerAll() {
        try {
            // Register all subsystems
            discoverAndRegister(SUBSYSTEM_PACKAGE, SubsystemBase.class, this::registerSubsystem);
            
            // Register all commands
            discoverAndRegister(COMMAND_PACKAGE, Command.class, this::registerCommand);
            
        } catch (Exception e) {
            System.err.println("Error during auto-registration: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Discover classes in a package that implement a given interface/class
     */
    private <T> void discoverAndRegister(String packageName, Class<T> type, Consumer<Class<?>> registerFunction) 
            throws IOException, ClassNotFoundException {
        
        ClassLoader classLoader = Thread.currentThread().getContextClassLoader();
        String path = packageName.replace('.', '/');
        Enumeration<URL> resources = classLoader.getResources(path);
        
        List<Class<?>> classes = new ArrayList<>();
        
        while (resources.hasMoreElements()) {
            File directory = new File(resources.nextElement().getFile());
            if (directory.exists()) {
                findClasses(directory, packageName, type, classes);
            }
        }
        
        // Register each discovered class
        for (Class<?> clazz : classes) {
            if (!clazz.isInterface() && !java.lang.reflect.Modifier.isAbstract(clazz.getModifiers())) {
                registerFunction.accept(clazz);
            }
        }
    }

    /**
     * Find all classes in a directory that implement a given interface/class
     */
    private <T> void findClasses(File directory, String packageName, Class<T> type, List<Class<?>> classes) 
            throws ClassNotFoundException {
        
        if (!directory.exists()) {
            return;
        }
        
        File[] files = directory.listFiles();
        if (files == null) {
            return;
        }
        
        for (File file : files) {
            if (file.isDirectory()) {
                findClasses(file, packageName + "." + file.getName(), type, classes);
            } else if (file.getName().endsWith(".class")) {
                String className = packageName + '.' + file.getName().substring(0, file.getName().length() - 6);
                Class<?> clazz = Class.forName(className);
                
                // Check if the class is a subclass of the given type and not abstract
                if (type.isAssignableFrom(clazz) && !clazz.isInterface() && 
                    !java.lang.reflect.Modifier.isAbstract(clazz.getModifiers())) {
                    classes.add(clazz);
                }
            }
        }
    }

    /**
     * Register a subsystem using reflection
     */
    private void registerSubsystem(Class<?> clazz) {
        try {
            // Try to get the no-arg constructor first
            Constructor<?> constructor = clazz.getConstructor();
            Object instance = constructor.newInstance();
            
            // If we get here, it's a valid subsystem
            SubsystemBase subsystem = (SubsystemBase) instance;
            
            // Register with the subsystem manager
            subsystemManager.registerSubsystem(clazz, subsystem);
            
            System.out.println("Registered subsystem: " + clazz.getSimpleName());
            
        } catch (NoSuchMethodException e) {
            // Try constructor with SubsystemManager parameter
            try {
                Constructor<?> constructor = clazz.getConstructor(SubsystemManager.class);
                Object instance = constructor.newInstance(subsystemManager);
                
                SubsystemBase subsystem = (SubsystemBase) instance;
                subsystemManager.registerSubsystem(clazz, subsystem);
                
                System.out.println("Registered subsystem (with manager): " + clazz.getSimpleName());
                
            } catch (Exception ex) {
                System.err.println("Failed to register subsystem " + clazz.getName() + ": " + ex.getMessage());
            }
        } catch (Exception e) {
            System.err.println("Failed to register subsystem " + clazz.getName() + ": " + e.getMessage());
        }
    }

    /**
     * Register a command using reflection
     */
    private void registerCommand(Class<?> clazz) {
        try {
            // Skip if it's the CommandManager itself
            if (clazz == CommandManager.class) {
                return;
            }
            
            // Try to create an instance with default constructor first
            try {
                Command command = (Command) clazz.getConstructor().newInstance();
                registerCommandInstance(clazz, command);
                return;
            } catch (NoSuchMethodException e) {
                // Continue to next attempt
            }
            
            // Try with SubsystemManager parameter
            try {
                Command command = (Command) clazz.getConstructor(SubsystemManager.class).newInstance(subsystemManager);
                registerCommandInstance(clazz, command);
                return;
            } catch (NoSuchMethodException e) {
                // Continue to next attempt
            }
            
            // Try with CommandManager parameter
            try {
                Command command = (Command) clazz.getConstructor(CommandManager.class).newInstance(commandManager);
                registerCommandInstance(clazz, command);
                return;
            } catch (NoSuchMethodException e) {
                // Continue to next attempt
            }
            
            // Try with both SubsystemManager and CommandManager parameters
            try {
                Command command = (Command) clazz.getConstructor(SubsystemManager.class, CommandManager.class)
                        .newInstance(subsystemManager, commandManager);
                registerCommandInstance(clazz, command);
                return;
            } catch (NoSuchMethodException e) {
                // No matching constructor found
                System.err.println("No suitable constructor found for command: " + clazz.getName());
            }
            
        } catch (Exception e) {
            System.err.println("Failed to register command " + clazz.getName() + ": " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    private void registerCommandInstance(Class<?> clazz, Command command) {
        // Convert class name to snake_case for the command name
        String commandName = clazz.getSimpleName()
                .replaceAll("Command$", "")  // Remove 'Command' suffix if present
                .replaceAll("(?<=[a-z])([A-Z])", "_$1")  // Convert camelCase to snake_case
                .toLowerCase();
        
        commandManager.registerCommand(commandName, command);
        System.out.println("Registered command: " + commandName);
    }
}
