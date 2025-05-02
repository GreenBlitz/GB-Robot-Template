package frc.utils;

import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import java.awt.*;

public class CameraPoseCalculator {

	public static void main(String[] args) {
		SwingUtilities.invokeLater(CameraPoseCalculator::createAndShowGUI);
	}

	private static void createAndShowGUI() {
		JFrame frame = new JFrame("Camera Pose Calculator");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(450, 350);
		frame.setLocationRelativeTo(null);

		JPanel panel = new JPanel();
		panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
		panel.setBorder(BorderFactory.createEmptyBorder(15, 15, 15, 15));

		Font labelFont = new Font("SansSerif", Font.PLAIN, 14);
		Font fieldFont = new Font("SansSerif", Font.PLAIN, 14);

		JTextField[] fields = new JTextField[6];
		String[] labels = {"X", "Y", "Z", "Roll (°)", "Pitch (°)", "Yaw (°)"};
		for (int i = 0; i < labels.length; i++) {
			JPanel inputRow = new JPanel(new BorderLayout(5, 5));
			JLabel label = new JLabel(labels[i]);
			label.setFont(labelFont);
			JTextField field = new JTextField();
			field.setFont(fieldFont);
			fields[i] = field;

			inputRow.add(label, BorderLayout.WEST);
			inputRow.add(field, BorderLayout.CENTER);
			inputRow.setMaximumSize(new Dimension(Integer.MAX_VALUE, 30));
			panel.add(inputRow);
			panel.add(Box.createRigidArea(new Dimension(0, 8)));
		}

		JTextArea outputArea = new JTextArea(4, 30);
		outputArea.setEditable(false);
		outputArea.setFont(new Font("Monospaced", Font.PLAIN, 13));
		outputArea.setBackground(new Color(245, 245, 245));
		outputArea.setBorder(BorderFactory.createLineBorder(Color.LIGHT_GRAY));
		outputArea.setLineWrap(true);
		outputArea.setWrapStyleWord(true);

		panel.add(Box.createRigidArea(new Dimension(0, 15)));
		panel.add(new JLabel("Pose Output:"));
		panel.add(Box.createRigidArea(new Dimension(0, 5)));
		panel.add(outputArea);

		DocumentListener updateListener = new DocumentListener() {
			public void changedUpdate(DocumentEvent e) { updateOutput(); }
			public void removeUpdate(DocumentEvent e) { updateOutput(); }
			public void insertUpdate(DocumentEvent e) { updateOutput(); }

			void updateOutput() {
				try {
					double x = Double.parseDouble(fields[0].getText());
					double y = Double.parseDouble(fields[1].getText());
					double z = Double.parseDouble(fields[2].getText());
					double roll = Double.parseDouble(fields[3].getText());
					double pitch = Double.parseDouble(fields[4].getText());
					double yaw = Double.parseDouble(fields[5].getText());

					String output = String.format(
						"Translation: [%.2f, %.2f, %.2f]\nRotation (RPY): [%.2f°, %.2f°, %.2f°]",
						x, y, z, roll, pitch, yaw
					);
					outputArea.setText(output);
				} catch (NumberFormatException ex) {
					outputArea.setText("Please enter valid numeric values.");
				}
			}
		};

		for (JTextField field : fields) {
			field.getDocument().addDocumentListener(updateListener);
		}

		frame.setContentPane(panel);
		frame.setVisible(true);
	}

}
