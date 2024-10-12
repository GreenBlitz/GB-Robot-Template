package frc.robot.hardware.angleencoder;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.phoenix6.Phoenix6Device;

public class CANCoderEncoder extends Phoenix6Device implements IAngleEncoder {

	private final CANcoder encoder;

	public CANCoderEncoder(String logPath, CANcoder encoder) {
		super(logPath);
		this.encoder = encoder;
		encoder.optimizeBusUtilization();
	}

	@Override
	public void setPosition(Rotation2d position) {
		encoder.setPosition(position.getRotations());
	}

}
