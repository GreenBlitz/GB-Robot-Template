package frc.robot.hardware.rev.request;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Function;

public class SparkMaxRequestBuilder {

	public static SparkMaxRequest<Rotation2d> build(
		Rotation2d setPoint,
		SparkBase.ControlType controlType,
		int pidSlot,
		Function<Rotation2d, Double> feedforwardCalculator
	) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, feedforwardCalculator, Rotation2d::getRotations);
	}

	public static SparkMaxRequest<Rotation2d> build(Rotation2d setPoint, SparkBase.ControlType controlType, int pidSlot) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, Rotation2d::getRotations);
	}

	public static SparkMaxRequest<Double> build(Double setPoint, SparkBase.ControlType controlType, int pidSlot) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, setpoint -> setpoint);
	}

	public static SparkMaxVelocityRequest build(Rotation2d setPoint, int pidSlot, Function<Rotation2d, Double> feedforwardCalculator, Function<Rotation2d, Double> setPointToDoubleConverter){
		return new SparkMaxVelocityRequest(setPoint,pidSlot,feedforwardCalculator,setPointToDoubleConverter);
	}

	public static SparkMaxVelocityRequest build(Rotation2d setPoint, int pidSlot, Function<Rotation2d, Double> setPointToDoubleConverter){
		return new SparkMaxVelocityRequest(setPoint,pidSlot,setPointToDoubleConverter);
	}


}
