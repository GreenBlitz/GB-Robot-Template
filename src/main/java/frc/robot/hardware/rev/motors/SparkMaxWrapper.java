package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class SparkMaxWrapper extends SparkMax {

	public SparkMaxWrapper(SparkMaxDeviceID deviceID) {
		super(deviceID.id(), deviceID.type());

		super.getClosedLoopController().setIAccum(1);
		Logger.recordOutput("Before reset I", super.getClosedLoopController().getIAccum());
		super.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
		Logger.recordOutput("After reset I", super.getClosedLoopController().getIAccum());

	}

	public double getVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}

	public Rotation2d getVelocityAnglePerSecond() {
		return Rotation2d.fromRotations(Conversions.perMinuteToPerSecond(getEncoder().getVelocity()));
	}

}
