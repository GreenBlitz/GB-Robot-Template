package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.SteerInputsAutoLogged;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.Queue;

public class TalonFXSteer implements ISteer {

	private final TalonFXWrapper motor;
	private final TalonFXSteerSignals signals;

	private final PositionVoltage positionVoltageRequest;
	private final VoltageOut voltageRequest;

	private final Queue<Double> positionQueue;

	public TalonFXSteer(TalonFXSteerConstants constants) {
		this.motor = constants.getMotor();
		this.signals = constants.getSignals();

		this.positionVoltageRequest = new PositionVoltage(0).withEnableFOC(constants.getEnableFOC());
		this.voltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOC());

		this.positionQueue = PhoenixOdometryThread6328.getInstance()
			.registerLatencySignal(motor, signals.positionSignal(), signals.velocitySignal());
	}

	@Override
	public void setBrake(boolean brake) {
		NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		motor.setNeutralMode(neutralModeValue);
	}

	@Override
	public void resetToAngle(Rotation2d angle) {
		motor.setPosition(angle.getRotations());
	}


	@Override
	public void stop() {
		motor.stopMotor();
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setControl(voltageRequest.withOutput(voltage));
	}

	@Override
	public void setTargetAngle(Rotation2d angle) {
		motor.setControl(positionVoltageRequest.withPosition(angle.getRotations()));
	}


	@Override
	public void updateInputs(ModuleInputsContainer inputs) {
		SteerInputsAutoLogged steerInputs = inputs.getSteerMotorInputs();
		steerInputs.isConnected = PhoenixProUtils.checkWithRetry(
			() -> BaseStatusSignal.refreshAll(
				signals.positionSignal(),
				signals.velocitySignal(),
				signals.accelerationSignal(),
				signals.voltageSignal()
			),
			TalonFXSteerConstants.NUMBER_OF_STATUS_CODE_RETRIES
		);
		steerInputs.angle = Rotation2d.fromRotations(motor.getLatencyCompensatedPosition());
		steerInputs.velocity = Rotation2d.fromRotations(motor.getLatencyCompensatedVelocity());
		steerInputs.acceleration = Rotation2d.fromRotations(signals.accelerationSignal().getValue());
		steerInputs.voltage = signals.voltageSignal().getValue();
		steerInputs.angleOdometrySamples = positionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
		positionQueue.clear();
	}

}
