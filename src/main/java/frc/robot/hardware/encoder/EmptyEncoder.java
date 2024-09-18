package frc.robot.hardware.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.IDevice;
import frc.robot.hardware.signal.InputSignal;

public class EmptyEncoder implements IAngleEncoder, IDevice {

	@Override
	public void setPosition(Rotation2d position) {}
	
	@Override
	public boolean isConnected() {
		return false;
	}
	
	@Override
	public void updateSignals(InputSignal... signals) {
	
	}
}
