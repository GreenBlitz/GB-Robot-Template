package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import frc.robot.hardware.request.IRequest;

import java.util.function.Function;

public class SparkMaxRequest<T> implements IRequest<T> {

	private final CANSparkBase.ControlType controlType;
	private final int pidSlot;
	private final Function<T, Double> feedforwardCalculator;
	private final Function<T, Double> setPointToDoubleConverter;
	private T setPoint;

	SparkMaxRequest(
		T setPoint,
		CANSparkBase.ControlType controlType,
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

	SparkMaxRequest(T setPoint, CANSparkBase.ControlType controlType, int pidSlot, Function<T, Double> setPointToDoubleConverter) {
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

	public Double getSetPointAsDouble() {
		return setPointToDoubleConverter.apply(setPoint);
	}

	public CANSparkBase.ControlType getControlType() {
		return controlType;
	}

	public int getPidSlot() {
		return pidSlot;
	}

	public double getFeedforwardCalculation() {
		return feedforwardCalculator.apply(setPoint);
	}

}
