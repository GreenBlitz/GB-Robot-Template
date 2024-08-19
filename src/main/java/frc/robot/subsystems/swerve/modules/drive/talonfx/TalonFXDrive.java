package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.drive.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.Queue;

public class TalonFXDrive implements IDrive {

	private final TalonFXWrapper driveMotor;
	private final TalonFXDriveSignals signals;
	private final TalonFXDriveConstants constants;

	private final VelocityVoltage velocityVoltageRequest;
	private final VoltageOut voltageRequest;

	private final Queue<Double> drivePositionQueue;

	public TalonFXDrive(TalonFXDriveConstants constants) {
		this.driveMotor = constants.getMotor();
		this.signals = constants.getSignals();
		this.constants = constants;

		this.velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(constants.getEnableFOC());
		this.voltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOC());

		this.drivePositionQueue = PhoenixOdometryThread6328.getInstance()
			.registerLatencySignal(driveMotor, signals.positionSignal(), signals.velocitySignal());

		driveMotor.setPosition(0);
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return constants.getSysIdConfig();
	}

	@Override
	public void setBrake(boolean brake) {
		NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		driveMotor.setNeutralMode(neutralModeValue);
	}

	@Override
	public void stop() {
		driveMotor.stopMotor();
	}

	@Override
	public void setVoltage(double voltage) {
		driveMotor.setControl(voltageRequest.withOutput(voltage));
	}

	@Override
	public void setTargetVelocity(Rotation2d velocityPerSecond) {
		driveMotor.setControl(velocityVoltageRequest.withVelocity(velocityPerSecond.getRotations()));
	}

	@Override
	public void updateInputs(DriveInputsAutoLogged inputs) {
		//@formatter:off
		inputs.isConnected = BaseStatusSignal.refreshAll(
			signals.positionSignal(),
			signals.velocitySignal(),
			signals.accelerationSignal(),
			signals.voltageSignal(),
			signals.statorCurrentSignal()
		).isOK();
		//@formatter:on
		inputs.angle = Rotation2d.fromRotations(driveMotor.getLatencyCompensatedPosition());
		inputs.velocity = Rotation2d.fromRotations(driveMotor.getLatencyCompensatedVelocity());
		inputs.acceleration = Rotation2d.fromRotations(signals.accelerationSignal().getValue());
		inputs.current = signals.statorCurrentSignal().getValue();
		inputs.voltage = signals.voltageSignal().getValue();
		inputs.angleOdometrySamples = drivePositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
		drivePositionQueue.clear();
	}

}
