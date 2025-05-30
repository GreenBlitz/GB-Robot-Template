package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.module.factory.RealModuleConstants;
import frc.utils.Conversions;
import frc.utils.battery.BatteryUtil;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class Module extends GBSubsystem {

	private final ControllableMotor driveMotor;
	private final ControllableMotor steerMotor;
	private final IAngleEncoder encoder;

	private final ModuleRequests requests;
	private final ModuleSignals signals;

	private final SysIdCalibrator sysIdCalibrator;

	private final RealModuleConstants constants;

	private SwerveModuleState targetState;

	public Module(
		String logPath,
		ControllableMotor driveMotor,
		ControllableMotor steerMotor,
		IAngleEncoder encoder,
		ModuleRequests requests,
		ModuleSignals signals,
		SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo,
		RealModuleConstants constants
	) {
		super(logPath);

		this.driveMotor = driveMotor;
		this.steerMotor = steerMotor;
		this.encoder = encoder;

		this.requests = requests;
		this.signals = signals;

		this.sysIdCalibrator = new SysIdCalibrator(sysIdConfigInfo, this, this::setTargetDriveVoltage);

		this.constants = constants;

		this.targetState = new SwerveModuleState();
		setState(targetState, true);
	}

	@Override
	protected void subsystemPeriodic() {
		driveMotor.updateSimulation();
		steerMotor.updateSimulation();

		updateInputs();
	}

	private void updateInputs() {
		driveMotor.updateInputs(signals.driveVelocitySignal());
		driveMotor.updateInputs(signals.driveVoltageSignal());
		driveMotor.updateInputs(signals.driveCurrentSignal());

		steerMotor.updateInputs(signals.steerAngleSignal());
		steerMotor.updateInputs(signals.steerVoltageSignal());

		encoder.updateInputs(signals.encoderAngleSignal());
	}

	public void setState(SwerveModuleState state, boolean isOpenLoop) {
		if (isOpenLoop) {
			this.targetState = state;
			setTargetDriveVelocityOpenLoop(state.speedMetersPerSecond);
			pointToAngle(state.angle);
		} else {
			this.targetState = state;
			setTargetDriveVelocityCloseLoop(state.speedMetersPerSecond);
			pointToAngle(state.angle);
		}
	}

	public void setTargetDriveVelocityCloseLoop(double targetVelocityMPS) {
		targetState.speedMetersPerSecond = targetVelocityMPS;
		driveMotor.applyRequest(requests.driveVelocityRequest().withSetPoint(metersToAngle(targetVelocityMPS)));
	}

	public void setTargetDriveVelocityOpenLoop(double targetVelocityMPS) {
		targetState.speedMetersPerSecond = targetVelocityMPS;
		setTargetDriveVoltage(BatteryUtil.DEFAULT_VOLTAGE * (targetVelocityMPS / constants.maxDriveVelocityMPS()));
	}

	public void setTargetDriveVoltage(double voltage) {
		targetState.speedMetersPerSecond = (voltage / BatteryUtil.DEFAULT_VOLTAGE) * constants.maxDriveVelocityMPS();
		driveMotor.applyRequest(requests.driveVoltageRequest().withSetPoint(voltage));
	}

	public void pointToAngle(Rotation2d angle) {
		targetState.angle = angle;
		steerMotor.applyRequest(requests.steerAngleRequest().withSetPoint(Rotation2d.fromRadians(MathUtil.angleModulus(angle.getRadians()))));
	}


	public double getDriveVelocityMPS() {
		return angleToMeters(signals.driveVelocitySignal().getLatestValue());
	}

	public double getDriveVoltage() {
		return signals.driveVoltageSignal().getLatestValue();
	}

	public double getDriveCurrent() {
		return signals.driveCurrentSignal().getLatestValue();
	}

	public Rotation2d getSteerAngle() {
		return signals.steerAngleSignal().getLatestValue();
	}

	public Rotation2d getAbsolutAngle() {
		return signals.encoderAngleSignal().getLatestValue();
	}

	public double getSteerVoltage() {
		return signals.steerVoltageSignal().getLatestValue();
	}

	public SwerveModuleState getTargetState() {
		return targetState;
	}

	public ModuleSignals getSignals() {
		return signals;
	}

	public ModuleRequests getRequests() {
		return requests;
	}

	public SysIdCalibrator getSysIdCalibrator() {
		return sysIdCalibrator;
	}

	public boolean isAtVelocityMPS(double targetVelocity, double tolerance) {
		return signals.driveVelocitySignal().isNear(metersToAngle(targetVelocity), metersToAngle(tolerance));
	}

	public boolean isAtAngle(Rotation2d targetAngle, Rotation2d tolerance) {
		return signals.steerAngleSignal().isNear(targetAngle, tolerance);
	}

	public boolean isAtState(SwerveModuleState targetState, double velocityTolerance, Rotation2d angleTolerance) {
		return isAtVelocityMPS(targetState.speedMetersPerSecond, velocityTolerance) && isAtAngle(targetState.angle, angleTolerance);
	}

	private Rotation2d metersToAngle(double meters) {
		return Conversions.distanceToAngle(meters, constants.wheelDiameterMeters());
	}

	private double angleToMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, constants.wheelDiameterMeters());
	}

}
