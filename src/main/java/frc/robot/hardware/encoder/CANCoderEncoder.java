package frc.robot.hardware.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.IDevice;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public class CANCoderEncoder extends CTRED im IAngleEncoder {

	private final CANcoder encoder;
	
	public CANCoderEncoder(CANcoder encoder, double frequency) {
		this.encoder = encoder;
	}

	@Override
	public void setPosition(Rotation2d position) {
		encoder.setPosition(position.getRotations());
	}
	@Override
	public boolean isConnected() {
		return BaseStatusSignal.isAllGood(encoder.getPosition());
	}
	
	@Override
	public void updateSignals(InputSignal... signals) {
		StatusSignal<>
		for (InputSignal signal : signals) {
			if (signal instanceof Phoenix6SignalBuilder.SignalGetter) {
				BaseStatusSignal.refreshAll(((Phoenix6SignalBuilder.SignalGetter) signal).getSignal());
			}
		}
		
	}

}
