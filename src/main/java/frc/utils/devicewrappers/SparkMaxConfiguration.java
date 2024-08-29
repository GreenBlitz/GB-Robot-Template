package frc.utils.devicewrappers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.PIDObject;

public class SparkMaxConfiguration {

	public double positionConversionFactor;
	public double velocityConversionFactor;
	public Rotation2d forwardAngleLimit;
	public Rotation2d backwardAngleLimit;
	public boolean enableForwardSoftLimit;
	public boolean enableBackwardSoftLimit;
	public PIDObject slot0;
	public boolean enableSlot0;
	public PIDObject slot1;
	public boolean enableSlot1;
	public PIDObject slot2;
	public boolean enableSlot2;
	public PIDObject slot3;
	public boolean enableSlot3;
	public int currentLimit;

	public SparkMaxConfiguration() {
		this.positionConversionFactor = 1;
		this.velocityConversionFactor = 1;

		this.forwardAngleLimit = Rotation2d.fromDegrees(0);
		this.backwardAngleLimit = Rotation2d.fromDegrees(0);
		this.enableForwardSoftLimit = false;
		this.enableBackwardSoftLimit = false;

		this.currentLimit = 0;

		this.slot0 = new PIDObject();
		this.enableSlot0 = false;
		this.slot1 = new PIDObject();
		this.enableSlot1 = false;
		this.slot2 = new PIDObject();
		this.enableSlot2 = false;
		this.slot3 = new PIDObject();
		this.enableSlot3 = false;
	}

	public SparkMaxConfiguration withPositionConversionFactor(double conversionFactor) {
		this.positionConversionFactor = conversionFactor;
		return this;
	}

	public SparkMaxConfiguration withVelocityConversionFactor(double conversionFactor){
		this.velocityConversionFactor = conversionFactor;
		return this;
	}

	public SparkMaxConfiguration withForwardAngleLimit(Rotation2d forwardAngleLimit) {
		this.forwardAngleLimit = forwardAngleLimit;
		return this;
	}

	public SparkMaxConfiguration withBackwardAngleLimit(Rotation2d backwardAngleLimit) {
		this.backwardAngleLimit = backwardAngleLimit;
		return this;
	}

	public SparkMaxConfiguration withForwardEnableSoftLimit(boolean forwardEnableSoftLimit) {
		this.enableForwardSoftLimit = forwardEnableSoftLimit;
		return this;
	}

	public SparkMaxConfiguration withBackwardEnableSoftLimit(boolean backwardEnableSoftLimit) {
		this.enableBackwardSoftLimit = backwardEnableSoftLimit;
		return this;
	}

	public SparkMaxConfiguration withCurrentLimit(int limit) {
		this.currentLimit = limit;
		return this;
	}

	public SparkMaxConfiguration withSlot0(PIDObject slot0) {
		this.slot0 = new PIDObject(slot0);
		return this;
	}

	public SparkMaxConfiguration withSlot1(PIDObject slot1) {
		this.slot1 = new PIDObject(slot1);
		return this;
	}

	public SparkMaxConfiguration withSlot2(PIDObject slot2) {
		this.slot2 = new PIDObject(slot2);
		return this;
	}

	public SparkMaxConfiguration withSlot3(PIDObject slot3) {
		this.slot3 = new PIDObject(slot3);
		return this;
	}

}
