package frc.utils.mech2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.HashMap;

public class Mechanism2dContainer {

	private final Mechanism2d mechanism2d;
	private HashMap<String , MechanismLigament2d> ligaments = new HashMap<String, MechanismLigament2d>();
	private String name;

	public Mechanism2dContainer(Mechanism2d mechanism2d, String name) {
		this.mechanism2d = mechanism2d;
		this.name = name;
	}

	public MechanismRoot2d getExistingRoot(String name) {
		return mechanism2d.getRoot(name, 0, 0);
	}

	public void addRoot(String name, double x, double y) {
		mechanism2d.getRoot(name, x, y);
	}

	public void addLigament(String root, MechanismLigament2d ligament) {
		getExistingRoot(root).append(ligament);
		ligaments.put(ligament.getName(), ligament);
	}

	public void extendLigament(String addToLigament, MechanismLigament2d ligment) {
		ligaments.get(addToLigament).append(ligment);
		ligaments.put(ligment.getName(), ligment);

	}

	public void setRootPosition(String root, double x, double y) {
		getExistingRoot(root).setPosition(x, y);
	}

	public void setLigamentAngle(String ligament, Rotation2d angle) {
		ligaments.get(ligament).setAngle(angle);
	}

	public Rotation2d getLigamentAngle(String ligament) {
		return Rotation2d.fromDegrees(ligaments.get(ligament).getAngle());
	}

	public void setLigamentLength(String ligament, double length) {
		ligaments.get(ligament).setLength(length);
	}

	public double getLigamentLength(String ligament) {
		return ligaments.get(ligament).getLength();
	}

	public void setLigamentColor(String ligament, Color8Bit color) {
		ligaments.get(ligament).setColor(color);
	}

	public Color8Bit getLigamentColor(String ligament) {
		return ligaments.get(ligament).getColor();
	}

	public void setLigamentWidth(String ligament, double width) {
		ligaments.get(ligament).setLineWeight(width);
	}

	public double getLigamentWidth(String ligament) {
		return ligaments.get(ligament).getLineWeight();
	}

	public void publish() {
		SmartDashboard.putData(name, mechanism2d);
	}

}
