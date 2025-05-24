package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;
import frc.utils.battery.BatteryUtil;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class Module extends GBSubsystem {

	private final ControllableMotor driveMotor;
	private final ControllableMotor steerMotor;

	private final ModuleRequests requests;
	private final ModuleSignals signals;

	private final SysIdCalibrator sysIdCalibrator;

	private final double maxDriveVelocityMPS;
	private final double wheelDiameterMeters;

	private SwerveModuleState targetState;

	public Module(
		String logPath,
		ControllableMotor driveMotor,
		ControllableMotor steerMotor,
		ModuleRequests requests,
		ModuleSignals signals,
		SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo,
		double maxDriveVelocityMPS,
		double wheelDiameterMeters
	) {
		super(logPath);

		this.driveMotor = driveMotor;
		this.steerMotor = steerMotor;

		this.requests = requests;
		this.signals = signals;

		this.sysIdCalibrator = new SysIdCalibrator(sysIdConfigInfo, this, this::setTargetDriveVoltage);

		this.maxDriveVelocityMPS = maxDriveVelocityMPS;
		this.wheelDiameterMeters = wheelDiameterMeters;

		this.targetState = new SwerveModuleState();
		setStateCloseLoop(targetState);
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
	}

	public void setStateCloseLoop(SwerveModuleState state) {
		this.targetState = state;
		setTargetDriveVelocityMPSCloseLoop(state.speedMetersPerSecond);
		pointToAngle(state.angle);
	}

	public void setStateOpenLoop(SwerveModuleState state) {
		this.targetState = state;
		setTargetDriveVelocityMPSOpenLoop(state.speedMetersPerSecond);
		pointToAngle(state.angle);
	}

	public void setTargetDriveVelocityMPSCloseLoop(double targetVelocity) {
		targetState.speedMetersPerSecond = targetVelocity;
		driveMotor.applyRequest(requests.driveVelocityRequest().withSetPoint(metersToAngle(targetVelocity)));
	}

	public void setTargetDriveVelocityMPSOpenLoop(double targetVelocity) {
		targetState.speedMetersPerSecond = targetVelocity;
		setTargetDriveVoltage(BatteryUtil.DEFAULT_VOLTAGE * (targetVelocity / maxDriveVelocityMPS));
	}

	public void setTargetDriveVoltage(double voltage) {
		targetState.speedMetersPerSecond = (voltage / BatteryUtil.getCurrentVoltage()) * maxDriveVelocityMPS;
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
		return Conversions.distanceToAngle(meters, wheelDiameterMeters);
	}

	private double angleToMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, wheelDiameterMeters);
	}

}
