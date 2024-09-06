package frc.robot.hardware.request.angle;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Consumer;

public class TalonFXAngleRequest implements IAngleRequest {

	private final Consumer<Rotation2d> withSetPoint;
	private final ControlRequest controlRequest;

	public TalonFXAngleRequest(ControlRequest controlRequest, Consumer<Rotation2d> withSetPoint) {
		this.withSetPoint = withSetPoint;
		this.controlRequest = controlRequest;
	}

	public TalonFXAngleRequest(PositionVoltage positionVoltage) {
		this(positionVoltage, setPoint -> positionVoltage.withPosition(setPoint.getRotations()));
	}

	public TalonFXAngleRequest(PositionTorqueCurrentFOC positionTorqueCurrentFOC) {
		this(positionTorqueCurrentFOC, setPoint -> positionTorqueCurrentFOC.withPosition(setPoint.getRotations()));
	}

	public TalonFXAngleRequest(MotionMagicExpoVoltage motionMagicExpoVoltage) {
		this(motionMagicExpoVoltage, setPoint -> motionMagicExpoVoltage.withPosition(setPoint.getRotations()));
	}

	public TalonFXAngleRequest(MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC) {
		this(motionMagicExpoTorqueCurrentFOC, setPoint -> motionMagicExpoTorqueCurrentFOC.withPosition(setPoint.getRotations()));
	}

	public TalonFXAngleRequest(VelocityVoltage velocityVoltage) {
		this(velocityVoltage, setPoint -> velocityVoltage.withVelocity(setPoint.getRotations()));
	}

	public TalonFXAngleRequest(VelocityTorqueCurrentFOC velocityTorqueCurrentFOC) {
		this(velocityTorqueCurrentFOC, setPoint -> velocityTorqueCurrentFOC.withVelocity(setPoint.getRotations()));
	}

	public TalonFXAngleRequest(MotionMagicVelocityVoltage motionMagicVelocityVoltage) {
		this(motionMagicVelocityVoltage, setPoint -> motionMagicVelocityVoltage.withVelocity(setPoint.getRotations()));
	}

	public TalonFXAngleRequest(MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityTorqueCurrentFOC) {
		this(motionMagicVelocityTorqueCurrentFOC, setPoint -> motionMagicVelocityTorqueCurrentFOC.withVelocity(setPoint.getRotations()));
	}

	@Override
	public TalonFXAngleRequest withSetPoint(Rotation2d setPoint) {
		withSetPoint.accept(setPoint);
		return this;
	}

	public ControlRequest getControlRequest() {
		return controlRequest;
	}

}
