package frc.robot.hardware.rev.request;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;

import java.util.function.Function;

public class SparkMaxVelocityRequest extends SparkMaxRequest<Rotation2d> {


    SparkMaxVelocityRequest(Rotation2d setPoint, SparkBase.ControlType controlType, int pidSlot, Function<Rotation2d, Double> feedforwardCalculator, Function<Rotation2d, Double> setPointToDoubleConverter) {
        super(setPoint, controlType, pidSlot, feedforwardCalculator, setPointToDoubleConverter);
    }

    SparkMaxVelocityRequest(Rotation2d setPoint, SparkBase.ControlType controlType, int pidSlot, Function<Rotation2d, Double> setPointToDoubleConverter) {
        super(setPoint, controlType, pidSlot, setPointToDoubleConverter);
    }

    @Override
    public Rotation2d getSparkMaxCompatibleSetPoint() {
        return Rotation2d.fromRotations(Conversions.perSecondToPerMinute((this.setPoint).getRotations()));
    }
}
