package frc.robot.hardware.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.signal.InputSignal;

public class EmptyEncoder implements IAngleEncoder {

	@Override
	public void setPosition(Rotation2d position) {}

	@Override
	public boolean isOK() {
		return false;
	}

	@Override
	public void updateInputs(ConnectedInputAutoLogged inputs) {
		inputs.connected = isOK();
	}

	@Override
	public void updateSignals(InputSignal... signal) {}

}
