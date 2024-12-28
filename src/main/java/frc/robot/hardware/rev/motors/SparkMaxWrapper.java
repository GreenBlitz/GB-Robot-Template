package frc.robot.hardware.rev.motors;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.rev.REVUtils;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class SparkMaxWrapper extends SparkMax {

	private static final int DEFAULT_CONFIG_NUMBER_OF_TRIES = 1;

	public SparkMaxWrapper(SparkMaxDeviceID deviceID) {
		super(deviceID.id(), deviceID.type());

		super.getClosedLoopController().setIAccum(1);
		Logger.recordOutput("Before reset I", super.getClosedLoopController().getIAccum());
		applyConfiguration(new SparkMaxConfiguration().withResetMode(ResetMode.kResetSafeParameters));
		Logger.recordOutput("After reset I", super.getClosedLoopController().getIAccum());

	}

	public double getVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}

	public Rotation2d getVelocityAnglePerSecond() {
		return Rotation2d.fromRotations(Conversions.perMinuteToPerSecond(getEncoder().getVelocity()));
	}

	public REVLibError applyConfiguration(SparkMaxConfiguration configuration, int numberOfTries) {
		return REVUtils.checkWithRetry(
			() -> super.configure(configuration.getSparkMaxConfig(), configuration.getResetMode(), configuration.getPersistMode()),
			numberOfTries
		);
	}

	public REVLibError applyConfiguration(SparkMaxConfiguration configuration) {
		return applyConfiguration(configuration, DEFAULT_CONFIG_NUMBER_OF_TRIES);
	}

}
