package frc.utils.devicewrappers;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class SparkMaxConfiguration {

	public double conversionFactor;
	public Rotation2d forwardAngleLimit;
	public Rotation2d backwardAngleLimit;
	public boolean enableForwardSoftLimit;
	public boolean enableBackwardSoftLimit;
	public PIDSlot slot0;
	public PIDSlot slot1;
	public PIDSlot slot2;
	public PIDSlot slot3;
	public int currentLimit;


	public SparkMaxConfiguration() {
		this.conversionFactor = 1;
		this.forwardAngleLimit = Rotation2d.fromDegrees(0);
		this.backwardAngleLimit = Rotation2d.fromDegrees(0);
		this.enableForwardSoftLimit = false;
		this.enableBackwardSoftLimit = false;
		this.currentLimit = 0;
		this.slot0 = new PIDSlot();
		this.slot1 = new PIDSlot();
		this.slot2 = new PIDSlot();
		this.slot3 = new PIDSlot();
	}

	public SparkMaxConfiguration withConversionFactor(double conversionFactor) {
		this.conversionFactor = conversionFactor;
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

	public SparkMaxConfiguration withBackwardEnableSoftLimit(boolean backwardEnableSoftLimit){
		this.enableBackwardSoftLimit = backwardEnableSoftLimit;
		return this;
	}

	public SparkMaxConfiguration withCurrentLimit(int limit) {
		this.currentLimit = limit;
		return this;
	}

	public SparkMaxConfiguration withSlot0(PIDSlot slot0){
		this.slot0 = slot0;
		return this;
	}

	public SparkMaxConfiguration withSlot1(PIDSlot slot1){
		this.slot1 = slot1;
		return this;
	}

	public SparkMaxConfiguration withSlot2(PIDSlot slot2){
		this.slot2 = slot2;
		return this;
	}

	public SparkMaxConfiguration withSlot3(PIDSlot slot3){
		this.slot3 = slot3;
		return this;
	}

}
