package frc.robot.field.gui;

import frc.robot.field.PointOfInterest;
import frc.robot.field.FieldPOIManager;
import frc.robot.field.TargetPOIManager;
import frc.robot.field.POIOperations;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import javax.swing.Timer;
import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;

/**
 * Autonomous Simulator Dialog - Simulates autonomous behavior with POIs
 * Allows testing of autonomous strategies and AI decision making
 */
public class AutonomousSimulatorDialog extends JDialog {
    
    private FieldPOIManager fieldManager;
    private TargetPOIManager targetManager;
    
    // Simulation components
    private SimulationPanel simulationPanel;
    private ControlPanel controlPanel;
    private StatusPanel statusPanel;
    
    // Simulation state
    private Pose2d robotPose;
    private boolean isRunning = false;
    private double simulationTime = 0.0;
    private Timer simulationTimer;
    
    // AI components
    private AutonomousAI autonomousAI;
    
    public AutonomousSimulatorDialog(Frame owner, FieldPOIManager fieldManager, TargetPOIManager targetManager) {
        super(owner, "Autonomous Simulator", true);
        
        this.fieldManager = fieldManager;
        this.targetManager = targetManager;
        
        // Initialize simulation
        initializeSimulation();
        initializeGUI();
        
        setSize(1200, 800);
        setLocationRelativeTo(owner);
    }
    
    private void initializeSimulation() {
        // Start robot at center of field
        robotPose = new Pose2d(new Translation2d(8.27, 4.01), null);
        
        // Create AI
        autonomousAI = new AutonomousAI(fieldManager, targetManager);
        
        // Setup simulation timer (50Hz)
        simulationTimer = new Timer(20, e -> updateSimulation());
    }
    
    private void initializeGUI() {
        setLayout(new BorderLayout());
        
        // Create panels
        simulationPanel = new SimulationPanel();
        controlPanel = new ControlPanel();
        statusPanel = new StatusPanel();
        
        // Layout
        JPanel rightPanel = new JPanel(new BorderLayout());
        rightPanel.add(controlPanel, BorderLayout.CENTER);
        rightPanel.add(statusPanel, BorderLayout.SOUTH);
        
        add(simulationPanel, BorderLayout.CENTER);
        add(rightPanel, BorderLayout.EAST);
    }
    
    private void updateSimulation() {
        if (!isRunning) return;
        
        simulationTime += 0.02; // 20ms timestep
        
        // Update AI decision
        AutonomousAI.AIAction action = autonomousAI.update(robotPose, simulationTime);
        
        // Apply action
        applyAIAction(action);
        
        // Update displays
        simulationPanel.repaint();
        statusPanel.updateStatus();
        
        // Stop after 20 seconds
        if (simulationTime >= 20.0) {
            stopSimulation();
        }
    }
    
    private void stopSimulation() {
        isRunning = false;
        simulationTimer.stop();
    }
    
    private void applyAIAction(AutonomousAI.AIAction action) {
        switch (action.type) {
            case MOVE_TO_POSITION:
                // Move towards target position
                Translation2d target = action.targetPosition.getTranslation();
                Translation2d current = robotPose.getTranslation();
                Translation2d delta = target.minus(current);
                
                // Limit movement speed
                double maxSpeed = 2.0; // 2 m/s
                if (delta.getNorm() > maxSpeed * 0.02) {
                    delta = delta.div(delta.getNorm()).times(maxSpeed * 0.02);
                }
                
                robotPose = new Pose2d(current.plus(delta), null);
                break;
                
            case SCORE_FUEL:
                // Simulate scoring (no movement)
                statusPanel.addScore(5.0);
                break;
                
            case COLLECT_FUEL:
                // Simulate fuel collection
                statusPanel.setFuel(true);
                break;
                
            case WAIT:
                // No action
                break;
        }
    }
    
    // ========== INNER CLASSES ==========
    
    /**
     * Simulation visualization panel
     */
    private class SimulationPanel extends JPanel {
        private static final double SCALE = 40.0; // pixels per meter
        
        public SimulationPanel() {
            setBackground(Color.WHITE);
            setPreferredSize(new Dimension((int)(16.54 * SCALE), (int)(8.02 * SCALE)));
        }
        
        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2d = (Graphics2D) g.create();
            
            // Enable antialiasing
            g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            
            // Draw field
            drawField(g2d);
            
            // Draw POIs
            drawPOIs(g2d);
            
            // Draw robot
            drawRobot(g2d);
            
            // Draw AI plan
            drawAIPlan(g2d);
            
            g2d.dispose();
        }
        
        private void drawField(Graphics2D g2d) {
            // Field background
            g2d.setColor(new Color(220, 180, 120));
            g2d.fillRect(0, 0, getWidth(), getHeight());
            
            // Field border
            g2d.setColor(Color.BLACK);
            g2d.setStroke(new BasicStroke(3));
            g2d.drawRect(0, 0, getWidth(), getHeight());
            
            // Grid
            g2d.setColor(Color.LIGHT_GRAY);
            g2d.setStroke(new BasicStroke(1));
            for (int i = 1; i < 16; i++) {
                int x = (int) (i * SCALE);
                g2d.drawLine(x, 0, x, getHeight());
            }
            for (int i = 1; i < 8; i++) {
                int y = getHeight() - (int) (i * SCALE);
                g2d.drawLine(0, y, getWidth(), y);
            }
        }
        
        private void drawPOIs(Graphics2D g2d) {
            for (PointOfInterest poi : fieldManager.getAllActivePOIs()) {
                drawPOI(g2d, poi);
            }
        }
        
        private void drawPOI(Graphics2D g2d, PointOfInterest poi) {
            Translation2d pos = poi.getPosition();
            int x = (int) (pos.getX() * SCALE);
            int y = getHeight() - (int) (pos.getY() * SCALE);
            
            // Color based on type
            Color color = getColorForPOIType(poi.getType());
            
            // Draw POI
            g2d.setColor(color);
            g2d.fillOval(x - 8, y - 8, 16, 16);
            
            // Draw border
            g2d.setColor(Color.BLACK);
            g2d.setStroke(new BasicStroke(2));
            g2d.drawOval(x - 8, y - 8, 16, 16);
            
            // Draw ID
            g2d.setColor(Color.BLACK);
            g2d.setFont(new Font("Arial", Font.BOLD, 9));
            g2d.drawString(poi.getId(), x - 20, y - 12);
        }
        
        private void drawRobot(Graphics2D g2d) {
            Translation2d pos = robotPose.getTranslation();
            int x = (int) (pos.getX() * SCALE);
            int y = getHeight() - (int) (pos.getY() * SCALE);
            
            // Robot body
            g2d.setColor(Color.BLUE);
            g2d.fillRect(x - 15, y - 15, 30, 30);
            
            // Robot border
            g2d.setColor(Color.BLACK);
            g2d.setStroke(new BasicStroke(2));
            g2d.drawRect(x - 15, y - 15, 30, 30);
            
            // Direction indicator
            if (robotPose.getRotation() != null) {
                double angle = robotPose.getRotation().getRadians();
                int dx = (int) (Math.cos(angle) * 20);
                int dy = (int) (Math.sin(angle) * 20);
                g2d.setColor(Color.RED);
                g2d.drawLine(x, y, x + dx, y - dy);
            }
        }
        
        private void drawAIPlan(Graphics2D g2d) {
            if (autonomousAI != null) {
                List<Pose2d> plannedPath = autonomousAI.getPlannedPath();
                if (plannedPath.size() > 1) {
                    g2d.setColor(Color.GREEN);
                    g2d.setStroke(new BasicStroke(2, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 
                            0, new float[]{5, 5}, 0));
                    
                    for (int i = 1; i < plannedPath.size(); i++) {
                        Pose2d from = plannedPath.get(i - 1);
                        Pose2d to = plannedPath.get(i);
                        
                        int x1 = (int) (from.getTranslation().getX() * SCALE);
                        int y1 = getHeight() - (int) (from.getTranslation().getY() * SCALE);
                        int x2 = (int) (to.getTranslation().getX() * SCALE);
                        int y2 = getHeight() - (int) (to.getTranslation().getY() * SCALE);
                        
                        g2d.drawLine(x1, y1, x2, y2);
                    }
                }
            }
        }
        
        private Color getColorForPOIType(PointOfInterest.POIType type) {
            switch (type) {
                case APRIL_TAG: return Color.BLUE;
                case SCORING_LOCATION: return Color.RED;
                case FUEL_SOURCE: return Color.GREEN;
                case OBSTACLE: return Color.ORANGE;
                case CLIMBING_ZONE: return Color.MAGENTA;
                case FIELD_CENTER: return Color.GRAY;
                case FIELD_CORNER: return Color.LIGHT_GRAY;
                case ALLIANCE_PARTNER: return Color.CYAN;
                case OPPONENT_ROBOT: return Color.PINK;
                default: return Color.BLACK;
            }
        }
    }
    
    /**
     * Control panel for simulation
     */
    private class ControlPanel extends JPanel {
        private JButton startButton;
        private JButton stopButton;
        private JButton resetButton;
        private JSlider speedSlider;
        private JComboBox<String> aiModeComboBox;
        
        public ControlPanel() {
            setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
            setBorder(new TitledBorder("Simulation Control"));
            setPreferredSize(new Dimension(250, 0));
            
            initializeComponents();
            layoutComponents();
        }
        
        private void initializeComponents() {
            startButton = new JButton("Start");
            stopButton = new JButton("Stop");
            resetButton = new JButton("Reset");
            speedSlider = new JSlider(1, 10, 5);
            aiModeComboBox = new JComboBox<>(new String[]{"Basic", "Target-Based", "Path Planning", "AI Advanced"});
            
            // Setup listeners
            startButton.addActionListener(e -> startSimulation());
            stopButton.addActionListener(e -> stopSimulation());
            resetButton.addActionListener(e -> resetSimulation());
            aiModeComboBox.addActionListener(e -> changeAIMode());
        }
        
        private void layoutComponents() {
            // AI Mode
            JPanel modePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            modePanel.add(new JLabel("AI Mode:"));
            modePanel.add(aiModeComboBox);
            add(modePanel);
            
            add(Box.createVerticalStrut(10));
            
            // Control buttons
            JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            buttonPanel.add(startButton);
            buttonPanel.add(stopButton);
            buttonPanel.add(resetButton);
            add(buttonPanel);
            
            add(Box.createVerticalStrut(10));
            
            // Speed control
            JPanel speedPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            speedPanel.add(new JLabel("Speed:"));
            speedPanel.add(speedSlider);
            add(speedPanel);
            
            add(Box.createVerticalGlue());
        }
        
        private void startSimulation() {
            isRunning = true;
            simulationTimer.start();
            startButton.setEnabled(false);
            stopButton.setEnabled(true);
        }
        
        private void stopSimulation() {
            isRunning = false;
            simulationTimer.stop();
            startButton.setEnabled(true);
            stopButton.setEnabled(false);
        }
        
        private void resetSimulation() {
            stopSimulation();
            simulationTime = 0.0;
            robotPose = new Pose2d(new Translation2d(8.27, 4.01), null);
            statusPanel.reset();
            simulationPanel.repaint();
        }
        
        private void changeAIMode() {
            String mode = (String) aiModeComboBox.getSelectedItem();
            if (autonomousAI != null) {
                autonomousAI.setMode(mode);
            }
        }
    }
    
    /**
     * Status panel for simulation information
     */
    private class StatusPanel extends JPanel {
        private JLabel timeLabel;
        private JLabel positionLabel;
        private JLabel scoreLabel;
        private JLabel fuelLabel;
        private JLabel actionLabel;
        
        private double score = 0.0;
        private boolean hasFuel = false;
        
        public StatusPanel() {
            setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
            setBorder(new TitledBorder("Status"));
            
            initializeComponents();
            layoutComponents();
        }
        
        private void initializeComponents() {
            timeLabel = new JLabel("Time: 0.0s");
            positionLabel = new JLabel("Position: (8.27, 4.01)");
            scoreLabel = new JLabel("Score: 0.0");
            fuelLabel = new JLabel("Fuel: No");
            actionLabel = new JLabel("Action: None");
        }
        
        private void layoutComponents() {
            add(timeLabel);
            add(positionLabel);
            add(scoreLabel);
            add(fuelLabel);
            add(actionLabel);
        }
        
        public void updateStatus() {
            timeLabel.setText(String.format("Time: %.1fs", simulationTime));
            positionLabel.setText(String.format("Position: (%.2f, %.2f)", 
                    robotPose.getTranslation().getX(), robotPose.getTranslation().getY()));
            
            if (autonomousAI != null) {
                AutonomousAI.AIAction currentAction = autonomousAI.getCurrentAction();
                actionLabel.setText("Action: " + currentAction.type);
            }
        }
        
        public void addScore(double points) {
            score += points;
            scoreLabel.setText(String.format("Score: %.1f", score));
        }
        
        public void setFuel(boolean hasFuel) {
            this.hasFuel = hasFuel;
            fuelLabel.setText("Fuel: " + (hasFuel ? "Yes" : "No"));
        }
        
        public void reset() {
            score = 0.0;
            hasFuel = false;
            updateStatus();
            scoreLabel.setText("Score: 0.0");
            fuelLabel.setText("Fuel: No");
        }
    }
    
    /**
     * Simple AI for autonomous simulation
     */
    private static class AutonomousAI {
        private FieldPOIManager fieldManager;
        private TargetPOIManager targetManager;
        private String mode = "Basic";
        private AIAction currentAction;
        private List<Pose2d> plannedPath;
        
        public AutonomousAI(FieldPOIManager fieldManager, TargetPOIManager targetManager) {
            this.fieldManager = fieldManager;
            this.targetManager = targetManager;
            this.currentAction = new AIAction(AutonomousAI.ActionType.WAIT, null);
            this.plannedPath = new ArrayList<>();
        }
        
        public AIAction update(Pose2d robotPose, double time) {
            // Simple AI logic based on mode
            switch (mode) {
                case "Basic":
                    return basicAI(robotPose, time);
                case "Target-Based":
                    return targetBasedAI(robotPose, time);
                case "Path Planning":
                    return pathPlanningAI(robotPose, time);
                case "AI Advanced":
                    return advancedAI(robotPose, time);
                default:
                    return basicAI(robotPose, time);
            }
        }
        
        private AIAction basicAI(Pose2d robotPose, double time) {
            if (time < 5.0) {
                // Move to scoring location
                PointOfInterest target = fieldManager.getPOI("scoring_center");
                if (target != null) {
                    currentAction = new AIAction(ActionType.MOVE_TO_POSITION, target.getPose());
                }
            } else if (time < 10.0) {
                // Score
                currentAction = new AIAction(ActionType.SCORE_FUEL, null);
            } else {
                // Wait
                currentAction = new AIAction(ActionType.WAIT, null);
            }
            
            return currentAction;
        }
        
        private AIAction targetBasedAI(Pose2d robotPose, double time) {
            PointOfInterest bestTarget = targetManager.getBestShootingTarget(robotPose, true);
            if (bestTarget != null) {
                double distance = bestTarget.distanceTo(robotPose);
                if (distance > 1.5) {
                    currentAction = new AIAction(ActionType.MOVE_TO_POSITION, bestTarget.getPose());
                } else {
                    currentAction = new AIAction(ActionType.SCORE_FUEL, null);
                }
            }
            
            return currentAction;
        }
        
        private AIAction pathPlanningAI(Pose2d robotPose, double time) {
            // Simple path planning - move through multiple targets
            if (plannedPath.isEmpty()) {
                // Create a simple path
                plannedPath.add(new Pose2d(new Translation2d(4.0, 2.0), null));
                plannedPath.add(new Pose2d(new Translation2d(8.0, 4.0), null));
                plannedPath.add(new Pose2d(new Translation2d(12.0, 6.0), null));
            }
            
            // Move to next waypoint
            if (!plannedPath.isEmpty()) {
                Pose2d target = plannedPath.get(0);
                double distance = target.getTranslation().getDistance(robotPose.getTranslation());
                
                if (distance < 0.5) {
                    plannedPath.remove(0);
                    if (!plannedPath.isEmpty()) {
                        target = plannedPath.get(0);
                    } else {
                        currentAction = new AIAction(ActionType.WAIT, null);
                        return currentAction;
                    }
                }
                
                currentAction = new AIAction(ActionType.MOVE_TO_POSITION, target);
            }
            
            return currentAction;
        }
        
        private AIAction advancedAI(Pose2d robotPose, double time) {
            // More sophisticated AI would go here
            // For now, use target-based AI
            return targetBasedAI(robotPose, time);
        }
        
        public void setMode(String mode) {
            this.mode = mode;
            this.plannedPath.clear(); // Reset path when mode changes
        }
        
        public AIAction getCurrentAction() {
            return currentAction;
        }
        
        public List<Pose2d> getPlannedPath() {
            return new ArrayList<>(plannedPath);
        }
        
        public static class AIAction {
            public ActionType type;
            public Pose2d targetPosition;
            
            public AIAction(ActionType type, Pose2d targetPosition) {
                this.type = type;
                this.targetPosition = targetPosition;
            }
        }
        
        public enum ActionType {
            MOVE_TO_POSITION, SCORE_FUEL, COLLECT_FUEL, WAIT
        }
    }
}
