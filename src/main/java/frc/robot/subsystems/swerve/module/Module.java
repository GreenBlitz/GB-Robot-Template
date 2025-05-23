package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.battery.BatteryUtil;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class Module extends GBSubsystem {

	private final ControllableMotor driveMotor;
	private final ControllableMotor steerMotor;

	private final IRequest<Rotation2d> driveVelocityRequest;
	private final IRequest<Double> driveVoltageRequest;
	private final IRequest<Rotation2d> steerAngleRequest;

	private final AngleSignal driveVelocitySignal;
	private final AngleSignal steerAngleSignal;

	private final SysIdCalibrator sysIdCalibrator;

	private final Rotation2d maxDriveVelocityRotation2dPerSecond;

	public Module(
		String logPath,
		ControllableMotor driveMotor,
		ControllableMotor steerMotor,
		IRequest<Rotation2d> driveVelocityRequest,
		IRequest<Double> driveVoltageRequest,
		IRequest<Rotation2d> steerAngleRequest,
		AngleSignal driveVelocitySignal,
		AngleSignal steerAngleSignal,
		SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo,
		Rotation2d maxDriveVelocityRotation2dPerSecond
	) {
		super(logPath);

		this.driveMotor = driveMotor;
		this.steerMotor = steerMotor;

		this.driveVelocityRequest = driveVelocityRequest;
		this.driveVoltageRequest = driveVoltageRequest;
		this.steerAngleRequest = steerAngleRequest;

		this.driveVelocitySignal = driveVelocitySignal;
		this.steerAngleSignal = steerAngleSignal;

		this.sysIdCalibrator = new SysIdCalibrator(sysIdConfigInfo, this, this::setTargetDriveVoltage);

		this.maxDriveVelocityRotation2dPerSecond= maxDriveVelocityRotation2dPerSecond;
	}

	@Override
	protected void subsystemPeriodic() {
		driveMotor.updateSimulation();
		steerMotor.updateSimulation();

		updateInputs();
	}

	private void updateInputs() {
		driveMotor.updateInputs(driveVelocitySignal);
		steerMotor.updateInputs(steerAngleSignal);
	}

	public void setState(ModuleState state){
		setTargetDriveVelocityRotation2dPerSecondWithPID(state.getDriveVelocityRotation2dPerSecond());
		pointToAngle(state.getAngle());
	}


	public void setTargetDriveVelocityRotation2dPerSecondWithPID(Rotation2d targetVelocity) {
		driveMotor.applyRequest(driveVelocityRequest.withSetPoint(targetVelocity));
	}

	public void setTargetDriveVelocityRotation2dPerSecondWithoutPID(Rotation2d targetVelocity) {
		setTargetDriveVoltage(BatteryUtil.getCurrentVoltage() / (maxDriveVelocityRotation2dPerSecond.getRotations()/targetVelocity.getRotations()));
	}

	public void setTargetDriveVoltage(double voltage) {
		driveMotor.applyRequest(driveVoltageRequest.withSetPoint(voltage));
	}

	public void pointToAngle(Rotation2d angle) {
		steerMotor.applyRequest(steerAngleRequest.withSetPoint(Rotation2d.fromRadians(MathUtil.angleModulus(angle.getRadians()))));
	}


	public Rotation2d getDriveVelocityRotation2dPerSecond() {
		return driveVelocitySignal.getLatestValue();
	}

	public Rotation2d getSteerAngle() {
		return steerAngleSignal.getLatestValue();
	}

	public SysIdCalibrator getSysIdCalibrator() {
		return sysIdCalibrator;
	}
}
