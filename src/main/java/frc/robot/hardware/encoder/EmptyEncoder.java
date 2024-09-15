package frc.robot.hardware.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;

public class EmptyEncoder implements IAngleEncoder {

	@Override
	public void setPosition(Rotation2d position) {}

	@Override
	public boolean isConnected() {
		return false;
	}

	@Override
	public void updateInputs(ConnectedInputAutoLogged inputs) {
		inputs.connected = isConnected();
	}

}
