package frc.utils.devicewrappers;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class SparkMaxConfiguration {

	public double conversionFactor;
	public Rotation2d forwardAngleLimit;
	public Rotation2d backwardAngleLimit;
	public boolean enableSoftLimit;
	public Rotation2d startingPosition;
	public SparkPIDController pidController;
	public int currentLimit;

	public SparkMaxConfiguration() {
		this.conversionFactor = 1;
		this.forwardAngleLimit = Rotation2d.fromDegrees(0);
		this.backwardAngleLimit = Rotation2d.fromDegrees(0);
		this.enableSoftLimit = false;
		this.startingPosition = Rotation2d.fromDegrees(0);
		this.currentLimit = 0;
	}

	public SparkMaxConfiguration withConversionFactor(double newConversionFactor) {
		this.conversionFactor = newConversionFactor;
		return this;
	}

	public SparkMaxConfiguration withForwardAngleLimit(Rotation2d newForwardAngleLimit) {
		this.forwardAngleLimit = newForwardAngleLimit;
		return this;
	}

	public SparkMaxConfiguration withBackwardAngleLimit(Rotation2d newBackwardAngleLimit) {
		this.backwardAngleLimit = newBackwardAngleLimit;
		return this;
	}

	public SparkMaxConfiguration withEnableSoftLimit(boolean newEnableSoftLimit) {
		this.enableSoftLimit = newEnableSoftLimit;
		return this;
	}

	public SparkMaxConfiguration withStartingPosition(Rotation2d newStartingPosition) {
		this.startingPosition = newStartingPosition;
		return this;
	}

	public SparkMaxConfiguration withPIDController(SparkPIDController newPIDController) {
		this.pidController = newPIDController;
		return this;
	}

	public SparkMaxConfiguration withCurrentLimit(int newLimit) {
		this.currentLimit = newLimit;
		return this;
	}

}
