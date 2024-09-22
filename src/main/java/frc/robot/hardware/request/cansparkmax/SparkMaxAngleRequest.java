package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;

import java.util.function.Function;

public class SparkMaxAngleRequest implements IRequest<Rotation2d> {

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
	private final Function<CANSparkMax, Double> feedforwardCalculator;
	private Rotation2d setPoint;

	public SparkMaxAngleRequest(
		Rotation2d setPoint,
		SparkAngleRequestType controlType,
		int pidSlot,
		Function<CANSparkMax, Double> feedforwardCalculator
	) {
		this.setPoint = setPoint;
		this.controlType = controlType;
		this.pidSlot = pidSlot;
		this.feedforwardCalculator = feedforwardCalculator;
	}

	public SparkMaxAngleRequest(Rotation2d setPoint, SparkAngleRequestType controlType, int pidSlot) {
		this(setPoint, controlType, pidSlot, spark -> { return 0.0; });
	}

	@Override
	public SparkMaxAngleRequest withSetPoint(Rotation2d setPoint) {
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

	public Function<CANSparkMax, Double> getFeedforwardCalculator() {
		return feedforwardCalculator;
	}

}
