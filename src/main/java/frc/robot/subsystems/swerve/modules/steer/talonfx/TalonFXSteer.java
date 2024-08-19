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
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.Queue;

public class TalonFXSteer implements ISteer {

	private final TalonFXWrapper motor;
	private final TalonFXSteerSignals signals;
	private final TalonFXSteerConstants constants;

	private final PositionVoltage positionVoltageRequest;
	private final VoltageOut voltageRequest;

	private final Queue<Double> positionQueue;

	public TalonFXSteer(TalonFXSteerConstants constants) {
		this.motor = constants.getMotor();
		this.signals = constants.getSignals();
		this.constants = constants;

		this.positionVoltageRequest = new PositionVoltage(0).withEnableFOC(constants.getEnableFOC());
		this.voltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOC());

		this.positionQueue = PhoenixOdometryThread6328.getInstance()
			.registerLatencySignal(motor, signals.positionSignal(), signals.velocitySignal());
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return constants.getSysIdConfigInfo();
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
	public void updateInputs(SteerInputsAutoLogged inputs) {
		//@formatter:off
		inputs.isConnected = BaseStatusSignal.refreshAll(
			signals.positionSignal(),
			signals.velocitySignal(),
			signals.accelerationSignal(),
			signals.voltageSignal()
		).isOK();
		//@formatter:on
		inputs.angle = Rotation2d.fromRotations(motor.getLatencyCompensatedPosition());
		inputs.velocity = Rotation2d.fromRotations(motor.getLatencyCompensatedVelocity());
		inputs.acceleration = Rotation2d.fromRotations(signals.accelerationSignal().getValue());
		inputs.voltage = signals.voltageSignal().getValue();
		inputs.angleOdometrySamples = positionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
		positionQueue.clear();
	}

}
