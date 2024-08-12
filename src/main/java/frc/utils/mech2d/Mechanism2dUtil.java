package frc.utils.mech2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Mechanism2dUtil {

	private final Mechanism2d mechanism2d;
	private MechanismLigament2d[] ligaments;
	private int numLigaments;

	public Mechanism2dUtil(Mechanism2d mech2d) {
		this.mechanism2d = mech2d;
	}

	public MechanismRoot2d getExistingRoot(String name) {
		return mechanism2d.getRoot(name, 0, 0);
	}

	public void addRoot(String name, double x, double y) {
		mechanism2d.getRoot(name, x, y);
	}

	public void addLigament(String root, String name, double length, Rotation2d angle) {
		MechanismLigament2d ligament2d = new MechanismLigament2d(name, length, angle.getDegrees());
		getExistingRoot(root).append(ligament2d);
		ligaments[numLigaments] = ligament2d;
		numLigaments++;
	}

	public void addLigament(String root, String name, double length, Rotation2d angle, double width, Color8Bit color) {
		MechanismLigament2d ligament2d = new MechanismLigament2d(name, length, angle.getDegrees(), width, color);
		getExistingRoot(root).append(ligament2d);
		ligaments[numLigaments] = ligament2d;
		numLigaments++;
	}

	public void setRootPosition(String root, double x, double y) {
		getExistingRoot(root).setPosition(x, y);
	}

	public MechanismLigament2d getLigament(String ligament) {
		for (int i = 0; i <= numLigaments; i++) {
			if (ligaments[i].getName() == ligament) {
				return ligaments[i];
			}
		}
		return null;
	}

	public void setLigamentAngle(String ligament, Rotation2d angle) {
		getLigament(ligament).setAngle(angle);
	}

	public Rotation2d getLigamentAngle(String ligament) {
		Rotation2d newAngle = Rotation2d.fromDegrees(getLigamentAngle(ligament).getDegrees());
		return newAngle;
	}

	public void setLigamentLength(String ligament, double length) {
		getLigament(ligament).setLength(length);
	}

	public double getLigamentLength(String ligament) {
		return getLigament(ligament).getLength();
	}

	public void setLigamentColor(String ligament, Color8Bit color) {
		getLigament(ligament).setColor(color);
	}

	public Color8Bit getLigamentColor(String ligament) {
		return getLigament(ligament).getColor();
	}

	public void setLigamentWidth(String ligament, double width) {
		getLigament(ligament).setLineWeight(width);
	}

	public double getLigamentWidth(String ligament) {
		return getLigament(ligament).getLineWeight();
	}

	public void publish() {
		SmartDashboard.putData("Mechanism2d", mechanism2d);
	}

}
