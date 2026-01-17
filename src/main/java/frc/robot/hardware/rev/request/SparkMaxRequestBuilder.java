package frc.robot.hardware.rev.request;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Function;

public class SparkMaxRequestBuilder {

	public static SparkMaxRequest<Rotation2d> build(
		Rotation2d setPoint,
		SparkBase.ControlType controlType,
		ClosedLoopSlot pidSlot,
		Function<Rotation2d, Double> feedforwardCalculator
	) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, feedforwardCalculator, Rotation2d::getRotations);
	}

	public static SparkMaxRequest<Rotation2d> build(Rotation2d setPoint, SparkBase.ControlType controlType, ClosedLoopSlot pidSlot) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, Rotation2d::getRotations);
	}

	public static SparkMaxRequest<Double> build(Double setPoint, SparkBase.ControlType controlType, ClosedLoopSlot pidSlot) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, setpoint -> setpoint);
	}

}
