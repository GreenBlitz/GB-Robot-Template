package frc.robot.hardware.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;

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

}
