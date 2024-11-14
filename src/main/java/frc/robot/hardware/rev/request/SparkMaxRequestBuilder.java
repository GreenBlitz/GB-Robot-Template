package frc.robot.hardware.rev.request;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;

import java.util.function.Function;

public class SparkMaxRequestBuilder {

	public static SparkMaxRequest<Rotation2d> build(
		Rotation2d setPoint,
		CANSparkBase.ControlType controlType,
		int pidSlot,
		Function<Rotation2d, Double> feedforwardCalculator
	) {
		if (controlType == CANSparkBase.ControlType.kVelocity) {
			return new SparkMaxRequest<>(Rotation2d.fromRotations(Conversions.perSecondToPerMinute(setPoint.getRotations())), controlType, pidSlot, feedforwardCalculator, Rotation2d::getRotations);
		}
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, feedforwardCalculator, Rotation2d::getRotations);
	}

	public static SparkMaxRequest<Rotation2d> build(Rotation2d setPoint, CANSparkBase.ControlType controlType, int pidSlot) {
		if (controlType == CANSparkBase.ControlType.kVelocity) {
			return new SparkMaxRequest<>(Rotation2d.fromRotations(Conversions.perSecondToPerMinute(setPoint.getRotations())), controlType, pidSlot, Rotation2d::getRotations);
		}
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, Rotation2d::getRotations);
	}

	public static SparkMaxRequest<Double> build(Double setPoint, CANSparkBase.ControlType controlType, int pidSlot) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, setpoint -> setpoint);
	}

}
