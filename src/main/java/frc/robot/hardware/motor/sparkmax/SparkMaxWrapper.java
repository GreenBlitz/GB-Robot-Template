package frc.robot.hardware.motor.sparkmax;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;

public class SparkMaxWrapper extends CANSparkMax {

	public SparkMaxWrapper(SparkMaxDeviceID deviceID) {
		super(deviceID.ID(), deviceID.type());
		super.restoreFactoryDefaults();
	}

	public double getVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}
	public Rotation2d getVelocity() {
		return Rotation2d.fromRotations(Conversions.perMinuteToPerSecond(getEncoder().getVelocity()));
	}
}
