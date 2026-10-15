package frc.robot.hardware.rev.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxConfiguration {

	private SparkMaxConfig sparkMaxConfig;
	private ResetMode resetMode;
	private PersistMode persistMode;

	public SparkMaxConfiguration() {
		this.sparkMaxConfig = new SparkMaxConfig();
		this.resetMode = ResetMode.kNoResetSafeParameters;
		this.persistMode = PersistMode.kNoPersistParameters;
	}

	public SparkMaxConfig getSparkMaxConfig() {
		return sparkMaxConfig;
	}

	public ResetMode getResetMode() {
		return resetMode;
	}

	public PersistMode getPersistMode() {
		return persistMode;
	}

	public SparkMaxConfiguration withSparkMaxConfig(SparkMaxConfig sparkMaxConfig) {
		this.sparkMaxConfig = sparkMaxConfig;
		return this;
	}

	public SparkMaxConfiguration withResetMode(ResetMode resetMode) {
		this.resetMode = resetMode;
		return this;
	}

	public SparkMaxConfiguration withPersistMode(PersistMode persistMode) {
		this.persistMode = persistMode;
		return this;
	}

}
