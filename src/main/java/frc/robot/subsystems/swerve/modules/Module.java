package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.angleencoder.IAngleEncoder;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.modules.drive.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.SteerInputsAutoLogged;
import frc.utils.Conversions;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.calibration.sysid.SysIdCalibrator;

import java.util.Arrays;

public class Module {

	private final ModuleInputsContainer moduleInputsContainer;
	private final ControllableMotor iSteer;
	private final IRequest<Rotation2d> steerPositionRequest;
	private final IRequest<Double> steerVoltageRequest;
	private final ControllableMotor iDrive;
	private final IRequest<Rotation2d> driveVelocityRequest;
	private final IRequest<Double> driveVoltageRequest;
	private final IAngleEncoder angleEncoder;
	private final InputSignal<Rotation2d> encoderPositionSignal;
	private final ModuleConstants constants;

	private SwerveModuleState targetState;
	private Rotation2d startingSteerAngle;
	private boolean isClosedLoop;

	public Module(
		ModuleConstants constants,
		IAngleEncoder angleEncoder,
		InputSignal<Rotation2d> encoderPositionSignal,
		ControllableMotor iSteer,
		ControllableMotor iDrive
	) {
		this.constants = constants;
		this.angleEncoder = angleEncoder;
		this.encoderPositionSignal = encoderPositionSignal;
		this.iSteer = iSteer;
		this.iDrive = iDrive;
		this.moduleInputsContainer = new ModuleInputsContainer();

		this.targetState = new SwerveModuleState();
		this.startingSteerAngle = new Rotation2d();
		this.isClosedLoop = SwerveState.DEFAULT_DRIVE.getLoopMode().isClosedLoop;

		updateInputs();
		resetByEncoder();

		AlertManager.addAlert(
			new PeriodicAlert(Alert.AlertType.WARNING, constants.logPath() + "EncoderDisconnectAt", () -> !angleEncoder.isConnected())
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				constants.logPath() + "SteerDisconnectAt",
				() -> !moduleInputsContainer.getSteerMotorInputs().isConnected
			)
		);
		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				constants.logPath() + "DriveDisconnectAt",
				() -> !moduleInputsContainer.getDriveMotorInputs().isConnected
			)
		);
	}

	public SysIdCalibrator.SysIdConfigInfo getSteerSysIdConfigInfo() {
		return iSteer.getSysIdConfigInfo();
	}

	public SysIdCalibrator.SysIdConfigInfo getDriveSysIdConfigInfo() {
		return iDrive.getSysIdConfigInfo();
	}

	public double toDriveMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, constants.wheelDiameterMeters());
	}


	private void fixDriveInputsCoupling() {
		SteerInputsAutoLogged steerInputs = moduleInputsContainer.getSteerMotorInputs();
		DriveInputsAutoLogged driveInputs = moduleInputsContainer.getDriveMotorInputs();

		driveInputs.angle = ModuleUtils.getUncoupledAngle(driveInputs.angle, steerInputs.angle, constants.couplingRatio());
		driveInputs.velocity = ModuleUtils.getUncoupledAngle(driveInputs.velocity, steerInputs.velocity, constants.couplingRatio());
		driveInputs.acceleration = ModuleUtils.getUncoupledAngle(driveInputs.acceleration, steerInputs.acceleration, constants.couplingRatio());

		for (int i = 0; i < getNumberOfOdometrySamples(); i++) {
			Rotation2d steerDelta = Rotation2d
				.fromRotations(steerInputs.angleOdometrySamples[i].getRotations() - startingSteerAngle.getRotations());
			driveInputs.angleOdometrySamples[i] = ModuleUtils
				.getUncoupledAngle(driveInputs.angleOdometrySamples[i], steerDelta, constants.couplingRatio());
		}
	}

	public void updateInputs() {
		angleEncoder.updateSignals(encoderPositionSignal);
		iSteer.updateInputs(moduleInputsContainer.getSteerMotorInputs());
		iDrive.updateInputs(moduleInputsContainer.getDriveMotorInputs());
		fixDriveInputsCoupling();

		DriveInputsAutoLogged driveInputs = moduleInputsContainer.getDriveMotorInputs();
		driveInputs.distanceMeters = toDriveMeters(driveInputs.angle);
		driveInputs.velocityMeters = toDriveMeters(driveInputs.velocity);
		driveInputs.distanceMetersOdometrySamples = Arrays.stream(driveInputs.angleOdometrySamples).mapToDouble(this::toDriveMeters).toArray();

		ModuleInputsAutoLogged moduleInputs = moduleInputsContainer.getModuleInputs();
		moduleInputs.isAtTargetAngle = isAtTargetAngle();
		moduleInputs.isAtTargetVelocity = isAtTargetVelocity();
		moduleInputs.isClosedLoop = isClosedLoop;
		moduleInputs.targetState = targetState;

		moduleInputsContainer.processInputs(constants.logPath());
	}


	public void setClosedLoop(boolean closedLoop) {
		isClosedLoop = closedLoop;
	}

	public void setBrake(boolean brake) {
		iSteer.setBrake(brake);
		iDrive.setBrake(brake);
	}

	public void resetByEncoder() {
		startingSteerAngle = encoderPositionSignal.getLatestValue();
		iSteer.resetPosition(startingSteerAngle);
	}


	/**
	 * The odometry thread can update itself faster than the main code loop (which is 50 hertz). Instead of using the latest odometry update, the
	 * accumulated odometry positions since the last loop to get a more accurate position.
	 *
	 * @param odometryUpdateIndex the index of the odometry update
	 * @return the position of the module at the given odometry update index
	 */
	public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
		return new SwerveModulePosition(
			moduleInputsContainer.getDriveMotorInputs().distanceMetersOdometrySamples[odometryUpdateIndex],
			moduleInputsContainer.getSteerMotorInputs().angleOdometrySamples[odometryUpdateIndex]
		);
	}

	public int getNumberOfOdometrySamples() {
		return Math.min(
			moduleInputsContainer.getDriveMotorInputs().angleOdometrySamples.length,
			moduleInputsContainer.getSteerMotorInputs().angleOdometrySamples.length
		);
	}

	public SwerveModuleState getTargetState() {
		return targetState;
	}

	public SwerveModuleState getCurrentState() {
		return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getCurrentAngle());
	}

	public Rotation2d getDriveAngle() {
		return moduleInputsContainer.getDriveMotorInputs().angle;
	}

	public double getDriveVelocityMetersPerSecond() {
		return moduleInputsContainer.getDriveMotorInputs().velocityMeters;
	}

	public Rotation2d getCurrentAngle() {
		return moduleInputsContainer.getSteerMotorInputs().angle;
	}


	//@formatter:off
	public boolean isAtTargetVelocity() {
		return MathUtil.isNear(
			getTargetState().speedMetersPerSecond,
			getDriveVelocityMetersPerSecond(),
			ModuleConstants.SPEED_TOLERANCE_METERS_PER_SECOND
		);
	}
	//@formatter:on

	public boolean isAtTargetAngle() {
		boolean isStopping = moduleInputsContainer.getSteerMotorInputs().velocity.getRadians()
			<= ModuleConstants.ANGLE_VELOCITY_DEADBAND.getRadians();
		if (!isStopping) {
			return false;
		}
		boolean isAtAngle = MathUtil.isNear(
			MathUtil.angleModulus(getTargetState().angle.getRadians()),
			MathUtil.angleModulus(getCurrentAngle().getRadians()),
			ModuleConstants.ANGLE_TOLERANCE.getRadians()
		);
		return isAtAngle;
	}

	public boolean isAtTargetState() {
		return isAtTargetAngle() && isAtTargetVelocity();
	}


	public void stop() {
		targetState = new SwerveModuleState(0, moduleInputsContainer.getSteerMotorInputs().angle);
		iSteer.stop();
		iDrive.stop();
	}


	public void setDriveVoltage(double voltage) {
		setClosedLoop(false);
		iDrive.applyDoubleRequest(driveVoltageRequest.withSetPoint(voltage));
	}

	public void setSteerVoltage(double voltage) {
		iSteer.applyDoubleRequest(steerVoltageRequest.withSetPoint(voltage));
	}


	public void pointToAngle(Rotation2d angle, boolean optimize) {
		SwerveModuleState moduleState = new SwerveModuleState(0, angle);
		if (optimize) {
			targetState.angle = SwerveModuleState.optimize(moduleState, getCurrentAngle()).angle;
		} else {
			targetState.angle = moduleState.angle;
		}
		iSteer.applyAngleRequest(steerPositionRequest.withSetPoint(targetState.angle));
	}


	public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop) {
		this.targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
		iSteer.applyAngleRequest(steerPositionRequest.withSetPoint(this.targetState.angle));
		setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle, isClosedLoop);
	}

	public void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle, boolean isClosedLoop) {
		targetVelocityMetersPerSecond = ModuleUtils.reduceSkew(targetVelocityMetersPerSecond, targetSteerAngle, getCurrentAngle());

		if (isClosedLoop) {
			setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
		} else {
			setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
		}
	}

	public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
		setClosedLoop(true);
		Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(targetVelocityMetersPerSecond, constants.wheelDiameterMeters());
		Rotation2d coupledVelocityPerSecond = ModuleUtils
			.getCoupledAngle(targetVelocityPerSecond, moduleInputsContainer.getSteerMotorInputs().velocity, constants.couplingRatio());
		iDrive.applyAngleRequest(driveVelocityRequest.withSetPoint(coupledVelocityPerSecond));
	}

	public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
		setClosedLoop(false);
		double voltage = ModuleUtils.velocityToOpenLoopVoltage(
			targetVelocityMetersPerSecond,
			moduleInputsContainer.getSteerMotorInputs().velocity,
			constants.couplingRatio(),
			constants.velocityAt12VoltsPerSecond(),
			constants.wheelDiameterMeters(),
			ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
		);
		iDrive.applyDoubleRequest(driveVoltageRequest.withSetPoint(voltage));
	}

}
