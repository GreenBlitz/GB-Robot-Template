package frc.robot.hardware.rev.request;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import org.wpilib.math.geometry.Angle2d;

import java.util.function.Function;

public class SparkMaxRequestBuilder {

	public static SparkMaxRequest<Angle2d> build(
		Angle2d setPoint,
		SparkBase.ControlType controlType,
		ClosedLoopSlot pidSlot,
		Function<Angle2d, Double> feedforwardCalculator
	) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, feedforwardCalculator, Angle2d::getRotations);
	}

	public static SparkMaxRequest<Angle2d> build(Angle2d setPoint, SparkBase.ControlType controlType, ClosedLoopSlot pidSlot) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, Angle2d::getRotations);
	}

	public static SparkMaxRequest<Double> build(Double setPoint, SparkBase.ControlType controlType, ClosedLoopSlot pidSlot) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, setpoint -> setpoint);
	}

}
