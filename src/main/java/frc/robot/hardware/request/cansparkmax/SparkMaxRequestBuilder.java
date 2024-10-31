package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Function;

public class SparkMaxRequestBuilder {

	//@formatter:off
    public SparkMaxRequest<Rotation2d> generate(
        Rotation2d setPoint,
        CANSparkBase.ControlType controlType,
        int pidSlot,
        Function<Rotation2d, Double> feedforwardCalculator
    ) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, feedforwardCalculator, Rotation2d::getRotations);
	}
    //@formatter:on

	public SparkMaxRequest<Rotation2d> generate(Rotation2d setPoint, CANSparkBase.ControlType controlType, int pidSlot) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, Rotation2d::getRotations);
	}

	//@formatter:off
    public SparkMaxRequest<Double> generate(
        Double setPoint,
        CANSparkBase.ControlType controlType,
        int pidSlot,
        Function<Double, Double> feedforwardCalculator
    ) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, feedforwardCalculator, setpoint -> setpoint);
	}
    //@formatter:on

	public SparkMaxRequest<Double> generate(Double setPoint, CANSparkBase.ControlType controlType, int pidSlot) {
		return new SparkMaxRequest<>(setPoint, controlType, pidSlot, setpoint -> setpoint);
	}

}
