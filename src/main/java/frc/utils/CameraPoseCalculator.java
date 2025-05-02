package frc.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.VisionConstants;
import frc.utils.pose.PoseUtil;

import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import java.awt.*;

public class CameraPoseCalculator {

	private static NetworkTableEntry targetPoseInCameraSpace;

	public static void main(String[] args) {
		targetPoseInCameraSpace = NetworkTableInstance.getDefault().getTable(args[0]).getEntry("targetpose_cameraspace");
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
					Pose3d tagInCamera = PoseUtil.toPose3D(
						targetPoseInCameraSpace.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]),
						AngleUnit.DEGREES
					);
					double x = Double.parseDouble(fields[0].getText());
					double y = Double.parseDouble(fields[1].getText());
					double z = Double.parseDouble(fields[2].getText());
					double roll = Math.toRadians(Double.parseDouble(fields[3].getText()));
					double pitch = Math.toRadians(Double.parseDouble(fields[4].getText()));
					double yaw = Math.toRadians(Double.parseDouble(fields[5].getText()));

					Pose3d tagInRobot = new Pose3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));

					Transform3d cameraToTag = new Transform3d(
						tagInCamera.getTranslation(),
						tagInCamera.getRotation()
					);
					Transform3d tagToCamera = cameraToTag.inverse();

					Pose3d cameraInRobot = tagInRobot.transformBy(tagToCamera);

					String output = String.format(
						"Translation: [%.2f, %.2f, %.2f]\nRotation (RPY): [%.2f°, %.2f°, %.2f°]",
						cameraInRobot.getX(),
						cameraInRobot.getY(),
						cameraInRobot.getZ(),
						Math.toDegrees(cameraInRobot.getRotation().getX()),
						Math.toDegrees(cameraInRobot.getRotation().getY()),
						Math.toDegrees(cameraInRobot.getRotation().getZ())
					);
					outputArea.setText(output);
				} catch (NumberFormatException | NullPointerException ex) {
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
