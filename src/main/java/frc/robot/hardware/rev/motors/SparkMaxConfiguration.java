package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxConfiguration {

	private SparkMaxConfig sparkMaxConfig;
	private SparkBase.ResetMode resetMode;
	private SparkBase.PersistMode persistMode;

	public SparkMaxConfiguration() {
		this.sparkMaxConfig = new SparkMaxConfig();
		this.resetMode = SparkBase.ResetMode.kNoResetSafeParameters;
		this.persistMode = SparkBase.PersistMode.kNoPersistParameters;
	}

	public SparkMaxConfig getSparkMaxConfig() {
		return sparkMaxConfig;
	}

	public SparkBase.ResetMode getResetMode() {
		return resetMode;
	}

	public SparkBase.PersistMode getPersistMode() {
		return persistMode;
	}

	public SparkMaxConfiguration withSparkMaxConfig(SparkMaxConfig sparkMaxConfig) {
		this.sparkMaxConfig = sparkMaxConfig;
		return this;
	}

	public SparkMaxConfiguration withResetMode(SparkBase.ResetMode resetMode) {
		this.resetMode = resetMode;
		return this;
	}

	public SparkMaxConfiguration withPersistMode(SparkBase.PersistMode persistMode) {
		this.persistMode = persistMode;
		return this;
	}

}
