package frc.robot.hardware.rev.request;

import com.revrobotics.spark.SparkBase;
import frc.robot.hardware.interfaces.IRequest;

import java.util.function.Function;

public class SparkMaxRequest<T> implements IRequest<T> {

	protected final SparkBase.ControlType controlType;
	protected final int pidSlot;
	protected final Function<T, Double> feedforwardCalculator;
	protected final Function<T, Double> setPointToDoubleConverter;
	protected T setPoint;

	SparkMaxRequest(
		T setPoint,
		SparkBase.ControlType controlType,
		int pidSlot,
		Function<T, Double> feedforwardCalculator,
		Function<T, Double> setPointToDoubleConverter
	) {
		this.setPoint = setPoint;
		this.controlType = controlType;
		this.pidSlot = pidSlot;
		this.feedforwardCalculator = feedforwardCalculator;
		this.setPointToDoubleConverter = setPointToDoubleConverter;
	}

	SparkMaxRequest(T setPoint, SparkBase.ControlType controlType, int pidSlot, Function<T, Double> setPointToDoubleConverter) {
		this(setPoint, controlType, pidSlot, CANSparkMAX -> 0.0, setPointToDoubleConverter);
	}

	@Override
	public SparkMaxRequest<T> withSetPoint(T setPoint) {
		this.setPoint = setPoint;
		return this;
	}

	@Override
	public T getSetPoint() {
		return setPoint;
	}

	public Double getSparkMaxCompatibleSetPoint() {
		return setPointToDoubleConverter.apply(setPoint);
	}

	public SparkBase.ControlType getControlType() {
		return controlType;
	}

	public int getPidSlot() {
		return pidSlot;
	}

	public double getFeedforwardGain() {
		return feedforwardCalculator.apply(setPoint);
	}

}
