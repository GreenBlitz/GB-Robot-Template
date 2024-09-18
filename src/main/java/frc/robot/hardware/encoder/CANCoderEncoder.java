package frc.robot.hardware.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.IDevice;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6BothLatencySignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import org.littletonrobotics.junction.Logger;

import java.util.LinkedList;
import java.util.Locale;

public class CANCoderEncoder implements IAngleEncoder, IDevice {
	
	private final CANcoder encoder;
	private final ConnectedInputAutoLogged connectedInput;
	private final String logPath;
	
	public CANCoderEncoder(CANcoder encoder, String logPath) {
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
		LinkedList<StatusSignal<Double>> statusSignals = new LinkedList<>();
		for (InputSignal signal : signals) {
			if (signal instanceof Phoenix6SignalBuilder.SignalGetter) {
				statusSignals.add(((Phoenix6SignalBuilder.SignalGetter) signal).getSignal());
				if (signal instanceof Phoenix6BothLatencySignal) {
					statusSignals.add(((Phoenix6BothLatencySignal) signal).getSignalSlope());
				}
			}
		}
		connectedInput.connected = BaseStatusSignal.refreshAll(statusSignals.toArray(StatusSignal[]::new)).isOK();
		Logger.processInputs(logPath, connectedInput);
	}
	
}
