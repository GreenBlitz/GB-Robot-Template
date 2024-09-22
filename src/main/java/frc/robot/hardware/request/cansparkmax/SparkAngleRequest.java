package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;

import java.util.function.BiFunction;

public class SparkAngleRequest implements IRequest<Rotation2d> {

	public enum SparkAngleRequestType {

		POSITION(CANSparkBase.ControlType.kPosition),
		VELOCITY(CANSparkBase.ControlType.kVelocity);

		private final CANSparkBase.ControlType type;

		SparkAngleRequestType(CANSparkBase.ControlType type) {
			this.type = type;
		}

	}

	private final SparkAngleRequestType controlType;
	private final int pidSlot;
	private final BiFunction<Rotation2d, Rotation2d, Double> feedforwardCalculator;
	private Rotation2d setPoint;

	public SparkAngleRequest(
		Rotation2d setPoint,
		SparkAngleRequestType controlType,
		int pidSlot,
		BiFunction<Rotation2d, Rotation2d, Double> feedforwardCalculator
	) {
		this.setPoint = setPoint;
		this.controlType = controlType;
		this.pidSlot = pidSlot;
		this.feedforwardCalculator = feedforwardCalculator;
	}
	
	public SparkAngleRequest(
			Rotation2d setPoint,
			SparkAngleRequestType controlType,
			int pidSlot
	) {
		this.setPoint = setPoint;
		this.controlType = controlType;
		this.pidSlot = pidSlot;
		this.feedforwardCalculator = (position, velocity) -> {return 0.0;};
	}

	@Override
	public SparkAngleRequest withSetPoint(Rotation2d setPoint) {
		this.setPoint = setPoint;
		return this;
	}

	public Rotation2d getSetPoint() {
		return setPoint;
	}

	public CANSparkBase.ControlType getControlType() {
		return controlType.type;
	}

	public int getPidSlot() {
		return pidSlot;
	}

	public BiFunction<Rotation2d, Rotation2d, Double> getFeedforwardCalculator() {
		return feedforwardCalculator;
	}

}
