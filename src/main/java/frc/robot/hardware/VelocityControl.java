package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

public class VelocityControl {

	boolean isMM;
	int slot;
	Rotation2d targetVelocity;


	public int getSlot() {
		return slot;
	}

	public boolean isMM() {
		return isMM;
	}

	public Rotation2d getTargetVelocity() {
		return targetVelocity;
	}

}
