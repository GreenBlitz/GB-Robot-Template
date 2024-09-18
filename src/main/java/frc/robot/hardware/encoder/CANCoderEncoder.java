package frc.robot.hardware.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.IDevice;
import frc.robot.hardware.Phoenix6Device;
import frc.robot.hardware.signal.InputSignal;


public class CANCoderEncoder extends Phoenix6Device implements IAngleEncoder, IDevice {

	private final CANcoder encoder;
	private final ConnectedInputAutoLogged connectedInput;
	private final String logPath;

	public CANCoderEncoder(CANcoder encoder, String logPath) {
		super(logPath);
		this.encoder = encoder;
		this.connectedInput = new ConnectedInputAutoLogged();
		this.logPath = logPath;
	}

	@Override
	public void setPosition(Rotation2d position) {
		encoder.setPosition(position.getRotations());
	}

	@Override
	public boolean isConnected() {
		connectedInput.connected = BaseStatusSignal.isAllGood(encoder.getPosition());
		return connectedInput.connected;
	}

	@Override
	public void updateSignals(InputSignal... signals) {
		super.updateSignals(signals);
	}

}
