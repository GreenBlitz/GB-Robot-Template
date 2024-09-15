package frc.robot.hardware.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6BothLatencySignal;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class CANCoderEncoder implements IAngleEncoder {

	private final CANcoder encoder;

	public CANCoderEncoder(CANcoder encoder) {
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
	public void updateInputs(ConnectedInputAutoLogged inputs){
		inputs.connected = isConnected();
	}

	@Override
	public void updateSignals(InputSignal... signals) {
		for (InputSignal signal1: signals) {
			if (signal1 instanceof Phoenix6SignalBuilder.SignalGetter) {
				BaseStatusSignal.refreshAll(((Phoenix6DoubleSignal) signal1).getSignal());
			}
			else if (signal1 instanceof Phoenix6BothLatencySignal) {
				BaseStatusSignal.refreshAll(((Phoenix6DoubleSignal) signal1).getSignal());
			}
		}
	}

}
