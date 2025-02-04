package frc.robot.hardware.rev.request;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;

import java.util.function.Function;

public class SparkMaxVelocityRequest extends SparkMaxRequest<Rotation2d> {

	SparkMaxVelocityRequest(
		Rotation2d setPoint,
		int pidSlot,
		Function<Rotation2d, Double> feedforwardCalculator,
		Function<Rotation2d, Double> setPointToDoubleConverter
	) {
		super(setPoint, SparkBase.ControlType.kVelocity, pidSlot, feedforwardCalculator, setPointToDoubleConverter);
	}

	public SparkMaxVelocityRequest(Rotation2d setPoint, int pidSlot, Function<Rotation2d, Double> setPointToDoubleConverter) {
		super(setPoint, SparkBase.ControlType.kVelocity, pidSlot, setPointToDoubleConverter);
	}

	public SparkMaxVelocityRequest(Rotation2d setPoint, int pidSlot){
		super(setPoint, SparkBase.ControlType.kVelocity, pidSlot, (Rotation2d::getRotations));
	}

	@Override
	public Double getSparkMaxCompatibleSetPoint() {
		return Conversions.perSecondToPerMinute(setPointToDoubleConverter.apply(setPoint));
	}
}
