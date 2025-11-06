package frc.robot.hardware.rev.motors;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.rev.REVUtil;
import frc.utils.Conversions;

public class SparkMaxWrapper extends SparkMax {

	private static final int DEFAULT_APPLY_CONFIG_RETRIES = 1;

	public SparkMaxWrapper(SparkMaxDeviceID deviceID) {
		super(deviceID.id(), deviceID.type());
		applyConfiguration(new SparkMaxConfiguration().withResetMode(ResetMode.kResetSafeParameters));
	}

	public double getVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}

	public Rotation2d getVelocityAnglePerSecond() {
		return Rotation2d.fromRotations(Conversions.perMinuteToPerSecond(getEncoder().getVelocity()));
	}

	public REVLibError applyConfiguration(SparkMaxConfiguration configuration, int numberOfTries) {
		return REVUtil.checkWithRetry(
			() -> super.configure(configuration.getSparkMaxConfig(), configuration.getResetMode(), configuration.getPersistMode()),
			numberOfTries
		);
	}

	public REVLibError applyConfiguration(SparkMaxConfiguration configuration) {
		return applyConfiguration(configuration, DEFAULT_APPLY_CONFIG_RETRIES);
	}

}
