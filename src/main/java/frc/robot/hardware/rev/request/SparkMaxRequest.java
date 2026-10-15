package frc.robot.hardware.rev.request;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;
import frc.utils.Conversions;

import java.util.function.Function;

public class SparkMaxRequest<T> implements IRequest<T> {

	private final SparkBase.ControlType controlType;
	private final ClosedLoopSlot pidSlot;
	private final Function<T, Double> feedforwardCalculator;
	private final Function<T, Double> setPointToDoubleConverter;
	private T setPoint;

	SparkMaxRequest(
		T setPoint,
		SparkBase.ControlType controlType,
		ClosedLoopSlot pidSlot,
		Function<T, Double> feedforwardCalculator,
		Function<T, Double> setPointToDoubleConverter
	) {
		this.setPoint = setPoint;
		this.controlType = controlType;
		this.pidSlot = pidSlot;
		this.feedforwardCalculator = feedforwardCalculator;
		this.setPointToDoubleConverter = setPointToDoubleConverter;
	}

	SparkMaxRequest(T setPoint, SparkBase.ControlType controlType, ClosedLoopSlot pidSlot, Function<T, Double> setPointToDoubleConverter) {
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
		boolean isVelocity = controlType == SparkBase.ControlType.kMAXMotionVelocityControl || controlType == SparkBase.ControlType.kVelocity;
		return setPointToDoubleConverter.apply(
			isVelocity ? (T) Rotation2d.fromRotations(Conversions.perSecondToPerMinute(((Rotation2d) setPoint).getRotations())) : setPoint
		);
	}

	public SparkBase.ControlType getControlType() {
		return controlType;
	}

	public ClosedLoopSlot getPidSlot() {
		return pidSlot;
	}

	public double getFeedforwardGain() {
		return feedforwardCalculator.apply(setPoint);
	}

}
