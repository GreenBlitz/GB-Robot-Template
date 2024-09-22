package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import frc.robot.hardware.request.IRequest;

import java.util.function.Function;

public class SparkDoubleRequest implements IRequest<Double> {

	public enum SparkDoubleRequestType {

		CURRENT(CANSparkBase.ControlType.kCurrent),
		VOLTAGE(CANSparkBase.ControlType.kVoltage);

		private final CANSparkBase.ControlType type;

		SparkDoubleRequestType(CANSparkBase.ControlType type) {
			this.type = type;
		}

	}

	private final SparkDoubleRequestType controlType;
	private final int pidSlot;
	private final Function<CANSparkMax, Double> feedforwardCalculator;
	private double setPoint;

	public SparkDoubleRequest(
		double setPoint,
		SparkDoubleRequestType controlType,
		int pidSlot,
		Function<CANSparkMax, Double> feedforwardCalculator
	) {
		this.setPoint = setPoint;
		this.controlType = controlType;
		this.pidSlot = pidSlot;
		this.feedforwardCalculator = feedforwardCalculator;
	}

	public SparkDoubleRequest(double setPoint, SparkDoubleRequestType controlType, int pidSlot) {
		this(setPoint, controlType, pidSlot, spark -> { return 0.0; });
	}

	@Override
	public SparkDoubleRequest withSetPoint(Double setPoint) {
		this.setPoint = setPoint;
		return this;
	}

	public double getSetPoint() {
		return setPoint;
	}

	public CANSparkBase.ControlType getControlType() {
		return controlType.type;
	}

	public int getPidSlot() {
		return pidSlot;
	}

	public Function<CANSparkMax, Double> getFeedforwardCalculator() {
		return feedforwardCalculator;
	}

}
