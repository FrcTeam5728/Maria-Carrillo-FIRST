package frc.robot.field.gui;

import frc.robot.field.PointOfInterest;
import frc.robot.field.FieldPOIManager;
import frc.robot.field.TargetPOIManager;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;
import edu.wpi.first.math.geometry.Translation2d;
import java.awt.Graphics2D;

/**
 * Field POI GUI - Interactive field editor for managing Points of Interest
 * Allows visual placement, tagging, and configuration of POIs
 */
public class FieldPOIGUI extends JFrame {
    
    // Field dimensions (in meters)
    private static final double FIELD_WIDTH = 16.54;
    private static final double FIELD_HEIGHT = 8.02;
    
    // GUI Components
    private FieldPanel fieldPanel;
    private POIListPanel poiListPanel;
    private ControlPanel controlPanel;
    
    // Data
    private FieldPOIManager fieldManager;
    private TargetPOIManager targetManager;
    private List<PointOfInterest> customPOIs;
    
    // GUI State
    private PointOfInterest selectedPOI;
    private boolean placingPOI = false;
    private PointOfInterest.POIType placingType;
    private double scale = 40.0; // pixels per meter
    
    public FieldPOIGUI() {
        super("Field POI Manager");
        
        // Initialize managers
        fieldManager = new FieldPOIManager(FIELD_WIDTH, FIELD_HEIGHT, Math.toRadians(60));
        targetManager = new TargetPOIManager(FIELD_WIDTH, FIELD_HEIGHT, Math.toRadians(60));
        customPOIs = new ArrayList<>();
        
        // Setup GUI
        initializeGUI();
        loadPOIs();
        
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(1400, 900);
        setLocationRelativeTo(null);
    }
    
    private void initializeGUI() {
        setLayout(new BorderLayout());
        
        // Create panels
        fieldPanel = new FieldPanel();
        poiListPanel = new POIListPanel();
        controlPanel = new ControlPanel();
        
        // Add panels
        add(fieldPanel, BorderLayout.CENTER);
        add(poiListPanel, BorderLayout.WEST);
        add(controlPanel, BorderLayout.EAST);
        
        // Setup listeners
        setupListeners();
    }
    
    private void setupListeners() {
        // Field panel mouse listener for POI placement
        fieldPanel.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                if (placingPOI) {
                    placePOI(e.getPoint());
                } else {
                    selectPOI(e.getPoint());
                }
            }
        });
        
        fieldPanel.addMouseMotionListener(new MouseAdapter() {
            @Override
            public void mouseMoved(MouseEvent e) {
                updateMousePosition(e.getPoint());
            }
        });
    }
    
    private void loadPOIs() {
        poiListPanel.refreshPOIList();
        fieldPanel.repaint();
    }
    
    private void placePOI(Point screenPoint) {
        Translation2d fieldPosition = screenToField(screenPoint);
        
        // Generate unique ID
        String id = "custom_" + (customPOIs.size() + 1);
        
        // Create POI
        PointOfInterest poi = new PointOfInterest(id, placingType, fieldPosition, 
                PointOfInterest.POIPriority.MEDIUM);
        
        // Add to managers
        fieldManager.addPOI(poi);
        customPOIs.add(poi);
        
        // Update GUI
        poiListPanel.refreshPOIList();
        fieldPanel.repaint();
        
        // Stop placing mode
        placingPOI = false;
        controlPanel.updatePlacingMode(false);
    }
    
    private void selectPOI(Point screenPoint) {
        Translation2d fieldPos = screenToField(screenPoint);
        
        // Find closest POI
        PointOfInterest closest = null;
        double minDistance = Double.MAX_VALUE;
        
        for (PointOfInterest poi : getAllPOIs()) {
            double distance = poi.getPosition().getDistance(fieldPos);
            if (distance < minDistance && distance < 0.5) { // Within 0.5 meters
                minDistance = distance;
                closest = poi;
            }
        }
        
        selectedPOI = closest;
        poiListPanel.selectPOI(selectedPOI);
        controlPanel.selectPOI(selectedPOI);
        fieldPanel.repaint();
    }
    
    private void updateMousePosition(Point screenPoint) {
        Translation2d fieldPos = screenToField(screenPoint);
        controlPanel.updateMousePosition(fieldPos);
    }
    
    private Translation2d screenToField(Point screenPoint) {
        double x = screenPoint.x / scale;
        double y = (fieldPanel.getHeight() - screenPoint.y) / scale;
        return new Translation2d(x, y);
    }
    
    private Point fieldToScreen(Translation2d fieldPos) {
        int x = (int) (fieldPos.getX() * scale);
        int y = (int) (fieldPanel.getHeight() - fieldPos.getY() * scale);
        return new Point(x, y);
    }
    
    private List<PointOfInterest> getAllPOIs() {
        List<PointOfInterest> allPOIs = new ArrayList<>();
        allPOIs.addAll(fieldManager.getAllActivePOIs());
        return allPOIs;
    }
    
    // ========== INNER CLASSES ==========
    
    /**
     * Field drawing panel
     */
    private class FieldPanel extends JPanel {
        
        public FieldPanel() {
            setBackground(Color.WHITE);
            setPreferredSize(new Dimension((int)(FIELD_WIDTH * scale), (int)(FIELD_HEIGHT * scale)));
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
            
            // Draw selection
            if (selectedPOI != null) {
                drawSelection(g2d, selectedPOI);
            }
            
            // Draw placing preview
            if (placingPOI) {
                drawPlacingPreview(g2d);
            }
            
            g2d.dispose();
        }
        
        private void drawField(Graphics2D g2d) {
            // Field background
            g2d.setColor(new Color(220, 180, 120)); // Beige field color
            g2d.fillRect(0, 0, getWidth(), getHeight());
            
            // Field border
            g2d.setColor(Color.BLACK);
            g2d.setStroke(new BasicStroke(3));
            g2d.drawRect(0, 0, getWidth(), getHeight());
            
            // Center line
            g2d.setColor(Color.WHITE);
            g2d.setStroke(new BasicStroke(2));
            int centerX = getWidth() / 2;
            g2d.drawLine(centerX, 0, centerX, getHeight());
            
            // Center circle
            int centerRadius = 30;
            g2d.drawOval(centerX - centerRadius, getHeight()/2 - centerRadius, 
                         centerRadius * 2, centerRadius * 2);
            
            // Grid lines (1 meter spacing)
            g2d.setColor(Color.LIGHT_GRAY);
            g2d.setStroke(new BasicStroke(1));
            for (int i = 1; i < FIELD_WIDTH; i++) {
                int x = (int) (i * scale);
                g2d.drawLine(x, 0, x, getHeight());
            }
            for (int i = 1; i < FIELD_HEIGHT; i++) {
                int y = getHeight() - (int) (i * scale);
                g2d.drawLine(0, y, getWidth(), y);
            }
        }
        
        private void drawPOIs(Graphics2D g2d) {
            for (PointOfInterest poi : getAllPOIs()) {
                drawPOI(g2d, poi);
            }
        }
        
        private void drawPOI(Graphics2D g2d, PointOfInterest poi) {
            Point screenPos = fieldToScreen(poi.getPosition());
            
            // Color based on type
            Color color = getColorForPOIType(poi.getType());
            
            // Size based on priority
            int size = getSizeForPriority(poi.getPriority());
            
            // Draw POI
            g2d.setColor(color);
            g2d.fillOval(screenPos.x - size/2, screenPos.y - size/2, size, size);
            
            // Draw border
            g2d.setColor(Color.BLACK);
            g2d.setStroke(new BasicStroke(2));
            g2d.drawOval(screenPos.x - size/2, screenPos.y - size/2, size, size);
            
            // Draw ID
            g2d.setColor(Color.BLACK);
            g2d.setFont(new Font("Arial", Font.BOLD, 10));
            FontMetrics fm = g2d.getFontMetrics();
            String id = poi.getId();
            int textWidth = fm.stringWidth(id);
            g2d.drawString(id, screenPos.x - textWidth/2, screenPos.y - size/2 - 2);
            
            // Draw interaction range
            if (poi.getType().isTarget()) {
                g2d.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), 50));
                int rangePixels = (int) (poi.getInteractionRange() * scale);
                g2d.fillOval(screenPos.x - rangePixels, screenPos.y - rangePixels, 
                             rangePixels * 2, rangePixels * 2);
            }
        }
        
        private void drawSelection(Graphics2D g2d, PointOfInterest poi) {
            Point screenPos = fieldToScreen(poi.getPosition());
            
            // Selection highlight
            g2d.setColor(Color.YELLOW);
            g2d.setStroke(new BasicStroke(3));
            int size = getSizeForPriority(poi.getPriority()) + 6;
            g2d.drawOval(screenPos.x - size/2, screenPos.y - size/2, size, size);
        }
        
        private void drawPlacingPreview(Graphics2D g2d) {
            // Get mouse position and draw preview
            Point mousePos = getMousePosition();
            if (mousePos != null) {
                Color color = getColorForPOIType(placingType);
                g2d.setColor(new Color(color.getRed(), color.getGreen(), color.getBlue(), 100));
                g2d.fillOval(mousePos.x - 10, mousePos.y - 10, 20, 20);
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
        
        private int getSizeForPriority(PointOfInterest.POIPriority priority) {
            switch (priority) {
                case CRITICAL: return 20;
                case HIGH: return 16;
                case MEDIUM: return 12;
                case LOW: return 8;
                default: return 10;
            }
        }
    }
    
    /**
     * POI list panel
     */
    private class POIListPanel extends JPanel {
        private JList<String> poiList;
        private DefaultListModel<String> listModel;
        
        public POIListPanel() {
            setLayout(new BorderLayout());
            setPreferredSize(new Dimension(250, 0));
            
            // Create components
            listModel = new DefaultListModel<>();
            poiList = new JList<>(listModel);
            JScrollPane scrollPane = new JScrollPane(poiList);
            
            // Add components
            add(scrollPane, BorderLayout.CENTER);
            
            // Setup listeners
            poiList.addListSelectionListener(e -> {
                if (!e.getValueIsAdjusting()) {
                    int index = poiList.getSelectedIndex();
                    if (index >= 0) {
                        selectPOIByIndex(index);
                    }
                }
            });
        }
        
        public void refreshPOIList() {
            listModel.clear();
            
            // Group POIs by type
            Map<PointOfInterest.POIType, List<PointOfInterest>> grouped = new HashMap<>();
            for (PointOfInterest poi : getAllPOIs()) {
                grouped.computeIfAbsent(poi.getType(), k -> new ArrayList<>()).add(poi);
            }
            
            // Add grouped POIs
            for (PointOfInterest.POIType type : grouped.keySet()) {
                listModel.addElement("--- " + type.getName() + " ---");
                for (PointOfInterest poi : grouped.get(type)) {
                    String info = String.format("%s (%.1f, %.1f)", 
                            poi.getId(), poi.getPosition().getX(), poi.getPosition().getY());
                    listModel.addElement("  " + info);
                }
            }
        }
        
        public void selectPOI(PointOfInterest poi) {
            if (poi == null) {
                poiList.clearSelection();
                return;
            }
            
            // Find and select in list
            for (int i = 0; i < listModel.size(); i++) {
                String item = listModel.getElementAt(i);
                if (item.contains(poi.getId())) {
                    poiList.setSelectedIndex(i);
                    poiList.ensureIndexIsVisible(i);
                    break;
                }
            }
        }
        
        private void selectPOIByIndex(int index) {
            String item = listModel.getElementAt(index);
            if (item.startsWith("  ")) { // It's a POI (not a header)
                // Extract POI ID
                String poiId = item.substring(2).split(" ")[0];
                selectedPOI = fieldManager.getPOI(poiId);
                controlPanel.selectPOI(selectedPOI);
                fieldPanel.repaint();
            }
        }
    }
    
    /**
     * Control panel
     */
    private class ControlPanel extends JPanel {
        private JLabel mousePosLabel;
        private JLabel selectedPOILabel;
        private JComboBox<PointOfInterest.POIType> typeComboBox;
        private JButton placePOIButton;
        private JButton deletePOIButton;
        private JButton saveButton;
        private JButton loadButton;
        private JButton simulateButton;
        
        public ControlPanel() {
            setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
            setPreferredSize(new Dimension(300, 0));
            
            // Create components
            createComponents();
            layoutComponents();
        }
        
        private void createComponents() {
            mousePosLabel = new JLabel("Mouse: (0.0, 0.0)");
            selectedPOILabel = new JLabel("Selected: None");
            
            typeComboBox = new JComboBox<>(PointOfInterest.POIType.values());
            
            placePOIButton = new JButton("Place POI");
            deletePOIButton = new JButton("Delete POI");
            saveButton = new JButton("Save POIs");
            loadButton = new JButton("Load POIs");
            simulateButton = new JButton("Simulate Autonomous");
            
            // Setup listeners
            placePOIButton.addActionListener(e -> startPlacingPOI());
            deletePOIButton.addActionListener(e -> deleteSelectedPOI());
            saveButton.addActionListener(e -> savePOIs());
            loadButton.addActionListener(e -> loadPOIs());
            simulateButton.addActionListener(e -> simulateAutonomous());
        }
        
        private void layoutComponents() {
            // Mouse position
            JPanel mousePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            mousePanel.add(mousePosLabel);
            add(mousePanel);
            
            // Selected POI
            JPanel selectedPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            selectedPanel.add(selectedPOILabel);
            add(selectedPanel);
            
            add(Box.createVerticalStrut(10));
            
            // POI placement
            JPanel placePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            placePanel.add(new JLabel("Type:"));
            placePanel.add(typeComboBox);
            add(placePanel);
            
            JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            buttonPanel.add(placePOIButton);
            add(buttonPanel);
            
            add(Box.createVerticalStrut(10));
            
            // POI management
            JPanel managePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            managePanel.add(deletePOIButton);
            add(managePanel);
            
            add(Box.createVerticalStrut(10));
            
            // File operations
            JPanel filePanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            filePanel.add(saveButton);
            filePanel.add(loadButton);
            add(filePanel);
            
            add(Box.createVerticalStrut(10));
            
            // Simulation
            JPanel simPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            simPanel.add(simulateButton);
            add(simPanel);
            
            add(Box.createVerticalGlue());
        }
        
        public void updateMousePosition(Translation2d pos) {
            mousePosLabel.setText(String.format("Mouse: (%.2f, %.2f)", pos.getX(), pos.getY()));
        }
        
        public void selectPOI(PointOfInterest poi) {
            if (poi == null) {
                selectedPOILabel.setText("Selected: None");
                deletePOIButton.setEnabled(false);
            } else {
                selectedPOILabel.setText(String.format("Selected: %s (%.2f, %.2f)", 
                        poi.getId(), poi.getPosition().getX(), poi.getPosition().getY()));
                deletePOIButton.setEnabled(true);
            }
        }
        
        public void updatePlacingMode(boolean isPlacing) {
            if (isPlacing) {
                placePOIButton.setText("Cancel Placing");
                placePOIButton.setBackground(Color.ORANGE);
            } else {
                placePOIButton.setText("Place POI");
                placePOIButton.setBackground(null);
            }
        }
        
        private void startPlacingPOI() {
            if (placingPOI) {
                placingPOI = false;
                updatePlacingMode(false);
            } else {
                placingType = (PointOfInterest.POIType) typeComboBox.getSelectedItem();
                placingPOI = true;
                updatePlacingMode(true);
            }
            fieldPanel.repaint();
        }
        
        private void deleteSelectedPOI() {
            if (selectedPOI != null && customPOIs.contains(selectedPOI)) {
                fieldManager.removePOI(selectedPOI.getId());
                customPOIs.remove(selectedPOI);
                selectedPOI = null;
                
                poiListPanel.refreshPOIList();
                selectPOI(null);
                fieldPanel.repaint();
            }
        }
        
        private void savePOIs() {
            JFileChooser chooser = new JFileChooser();
            chooser.setDialogTitle("Save POI Configuration");
            
            if (chooser.showSaveDialog(this) == JFileChooser.APPROVE_OPTION) {
                // TODO: Implement POI saving
                JOptionPane.showMessageDialog(this, "POI saving not yet implemented", 
                        "Not Implemented", JOptionPane.INFORMATION_MESSAGE);
            }
        }
        
        private void loadPOIs() {
            JFileChooser chooser = new JFileChooser();
            chooser.setDialogTitle("Load POI Configuration");
            
            if (chooser.showOpenDialog(this) == JFileChooser.APPROVE_OPTION) {
                // TODO: Implement POI loading
                JOptionPane.showMessageDialog(this, "POI loading not yet implemented", 
                        "Not Implemented", JOptionPane.INFORMATION_MESSAGE);
            }
        }
        
        private void simulateAutonomous() {
            // Open autonomous simulation dialog
            AutonomousSimulatorDialog dialog = new AutonomousSimulatorDialog(
                    FieldPOIGUI.this, fieldManager, targetManager);
            dialog.setVisible(true);
        }
    }
    
    // ========== MAIN METHOD ==========
    
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            try {
                // Set look and feel
                UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
            } catch (Exception e) {
                e.printStackTrace();
            }
            
            FieldPOIGUI gui = new FieldPOIGUI();
            gui.setVisible(true);
        });
    }
}
