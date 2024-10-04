package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import frc.robot.hardware.request.IRequest;

public class SparkMaxDoubleRequest implements IRequest<Double> {

	public enum SparkDoubleRequestType {

		CURRENT(CANSparkBase.ControlType.kCurrent),
		VOLTAGE(CANSparkBase.ControlType.kVoltage);

		private final CANSparkBase.ControlType type;

		SparkDoubleRequestType(CANSparkBase.ControlType type) {
			this.type = type;
		}

	}

	private final SparkDoubleRequestType controlType;
	private final int pidSlot;
	private double setPoint;

	public SparkMaxDoubleRequest(double setPoint, SparkDoubleRequestType controlType, int pidSlot) {
		this.setPoint = setPoint;
		this.controlType = controlType;
		this.pidSlot = pidSlot;
	}

	@Override
	public SparkMaxDoubleRequest withSetPoint(Double setPoint) {
		this.setPoint = setPoint;
		return this;
	}

	public double getSetPoint() {
		return setPoint;
	}

	public CANSparkBase.ControlType getControlType() {
		return controlType.type;
	}

	public int getPidSlot() {
		return pidSlot;
	}

}
