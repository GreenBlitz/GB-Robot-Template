package frc.robot.hardware.rev.request;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;

import java.util.function.Function;

public class SparkMaxVelocityRequest extends SparkMaxRequest<Rotation2d>{
    SparkMaxVelocityRequest(Rotation2d setPoint, int pidSlot, Function<Rotation2d, Double> feedforwardCalculator, Function<Rotation2d, Double> setPointToDoubleConverter) {
        super(setPoint, SparkBase.ControlType.kVelocity, pidSlot, feedforwardCalculator, setPointToDoubleConverter);
    }

    SparkMaxVelocityRequest(Rotation2d setPoint, int pidSlot, Function<Rotation2d, Double> setPointToDoubleConverter) {
        super(setPoint, SparkBase.ControlType.kVelocity, pidSlot, setPointToDoubleConverter);
    }
    public Double getSparkMaxCompatibleSetPoint() {
        double rotations = setPoint.getRotations();
        return setPointToDoubleConverter.apply(Rotation2d.fromRotations(Conversions.perSecondToPerMinute(rotations)));
    }


}
