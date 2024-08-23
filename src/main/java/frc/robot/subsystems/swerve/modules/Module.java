package frc.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPaths;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.MotorInputs;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.modules.drive.DriveThreadMetersInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.SteerThreadInputsAutoLogged;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Module {

	private final ModuleInputsContainer moduleInputsContainer;
	private final ISteer iSteer;
	private final IDrive iDrive;
	private final IEncoder iEncoder;
	private final ModuleConstants constants;

	private final VelocityVoltage driveVelocityControl;
	private final PositionVoltage steerPositionControl;

	private SwerveModuleState targetState;
	private Rotation2d startingSteerAngle;
	private boolean isClosedLoop;

	public Module(ModuleConstants constants, IEncoder iEncoder, ISteer iSteer, IDrive iDrive) {
		this.constants = constants;
		this.iEncoder = iEncoder;
		this.iSteer = iSteer;
		this.iDrive = iDrive;
		this.moduleInputsContainer = new ModuleInputsContainer();

		this.steerPositionControl = constants.steerPositionControl();
		this.driveVelocityControl = constants.driveVelocityControl();

		this.targetState = new SwerveModuleState();
		this.startingSteerAngle = new Rotation2d();
		this.isClosedLoop = SwerveState.DEFAULT_DRIVE.getLoopMode().isClosedLoop;

		logStatus();
		resetByEncoder();
		iDrive.resetAngle(new Rotation2d());
	}

	public SysIdCalibrator.SysIdConfigInfo getSteerSysidConfigInfo() {
		return iSteer.getSysidConfigInfo();
	}

	public SysIdCalibrator.SysIdConfigInfo getDriveSysidConfigInfo() {
		return iDrive.getSysidConfigInfo();
	}

	public double toDriveMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, constants.wheelDiameterMeters());
	}

	public void logStatus() {
		updateInputs();
		reportAlerts();
	}

	private void fixDriveInputsCoupling() {
		MotorInputs steerMotorInputs = moduleInputsContainer.getSteerMotorInputs();
		SteerThreadInputsAutoLogged steerThreadInputs = moduleInputsContainer.getSteerThreadInputs();
		MotorInputs driveMotorInputs = moduleInputsContainer.getDriveMotorInputs();
		DriveThreadMetersInputsAutoLogged driveThreadInputs = moduleInputsContainer.getDriveThreadMetersInputs();

		driveMotorInputs.angle = ModuleUtils.getUncoupledAngle(driveMotorInputs.angle, steerMotorInputs.angle, constants.couplingRatio());
		driveMotorInputs.velocity = ModuleUtils
			.getUncoupledAngle(driveMotorInputs.velocity, steerMotorInputs.velocity, constants.couplingRatio());
		driveMotorInputs.acceleration = ModuleUtils
			.getUncoupledAngle(driveMotorInputs.acceleration, steerMotorInputs.acceleration, constants.couplingRatio());

		for (int i = 0; i < driveThreadInputs.angleOdometrySamples.length; i++) {
			Rotation2d steerDelta = Rotation2d
				.fromRotations(steerThreadInputs.angleOdometrySamples[i].getRotations() - startingSteerAngle.getRotations());
			driveThreadInputs.angleOdometrySamples[i] = ModuleUtils
				.getUncoupledAngle(driveThreadInputs.angleOdometrySamples[i], steerDelta, constants.couplingRatio());
		}
	}

	public void updateInputs() {
		iEncoder.updateInputs(moduleInputsContainer.getEncoderInputs());
		iSteer.updateInputs(moduleInputsContainer.getSteerThreadInputs());
		iSteer.updateInputs(moduleInputsContainer.getSteerMotorInputs());
		iDrive.updateInputs(moduleInputsContainer.getDriveThreadMetersInputs());
		iDrive.updateInputs(moduleInputsContainer.getDriveMotorInputs());
		fixDriveInputsCoupling();

		DriveThreadMetersInputsAutoLogged driveInputs = moduleInputsContainer.getDriveThreadMetersInputs();
		MotorInputsAutoLogged driveMotorInputs = moduleInputsContainer.getDriveMotorInputs();
		driveInputs.distanceMeters = toDriveMeters(driveMotorInputs.angle);
		driveInputs.velocityMeters = toDriveMeters(driveMotorInputs.velocity);
		driveInputs.distanceMetersOdometrySamples = Arrays.stream(driveInputs.angleOdometrySamples).mapToDouble(this::toDriveMeters).toArray();

		ModuleInputsAutoLogged moduleInputs = moduleInputsContainer.getModuleInputs();
		moduleInputs.isAtTargetAngle = isAtTargetAngle();
		moduleInputs.isAtTargetVelocity = isAtTargetVelocity();
		moduleInputs.isClosedLoop = isClosedLoop;
		moduleInputs.targetState = targetState;

		moduleInputsContainer.processInputs(constants.logPath());
	}

	public void reportAlerts() {
		if (!moduleInputsContainer.getEncoderInputs().connected) {
			Logger.recordOutput(LogPaths.ALERT_LOG_PATH + constants.logPath() + "EncoderDisconnectAt", Timer.getFPGATimestamp());
		}
		if (!moduleInputsContainer.getSteerMotorInputs().connected) {
			Logger.recordOutput(LogPaths.ALERT_LOG_PATH + constants.logPath() + "SteerDisconnectAt", Timer.getFPGATimestamp());
		}
		if (!moduleInputsContainer.getDriveMotorInputs().connected) {
			Logger.recordOutput(LogPaths.ALERT_LOG_PATH + constants.logPath() + "DriveDisconnectAt", Timer.getFPGATimestamp());
		}
	}


	public void setClosedLoop(boolean closedLoop) {
		isClosedLoop = closedLoop;
	}

	public void setBrake(boolean isBrake) {
		iSteer.setBrake(isBrake);
		iDrive.setBrake(isBrake);
	}

	public void resetByEncoder() {
		startingSteerAngle = moduleInputsContainer.getEncoderInputs().angle;
		iSteer.resetAngle(startingSteerAngle);
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
			moduleInputsContainer.getDriveThreadMetersInputs().distanceMetersOdometrySamples[odometryUpdateIndex],
			moduleInputsContainer.getSteerThreadInputs().angleOdometrySamples[odometryUpdateIndex]
		);
	}

	public int getNumberOfOdometrySamples() {
		return moduleInputsContainer.getDriveThreadMetersInputs().distanceMetersOdometrySamples.length;
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
		return moduleInputsContainer.getDriveThreadMetersInputs().velocityMeters;
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
		iDrive.setVoltage(voltage);
	}

	public void setSteerVoltage(double voltage) {
		iSteer.setVoltage(voltage);
	}


	public void pointToAngle(Rotation2d angle, boolean optimize) {
		SwerveModuleState moduleState = new SwerveModuleState(0, angle);
		if (optimize) {
			targetState.angle = SwerveModuleState.optimize(moduleState, getCurrentAngle()).angle;
		} else {
			targetState.angle = moduleState.angle;
		}
		iSteer.setTargetAngle(
			new CloseLoopControl(steerPositionControl.withPosition(angle.getRotations()), steerPositionControl.Slot, angle, ControlState.PID)
		);
	}


	public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop) {
		this.targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
		iSteer.setTargetAngle(
			new CloseLoopControl(
				steerPositionControl.withPosition(this.targetState.angle.getRotations()),
				steerPositionControl.Slot,
				this.targetState.angle,
				ControlState.PID
			)
		);
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
		iDrive.setTargetVelocity(
			new CloseLoopControl(
				driveVelocityControl.withVelocity(coupledVelocityPerSecond.getRotations()),
				driveVelocityControl.Slot,
				coupledVelocityPerSecond,
				ControlState.PID
			)
		);
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
		iDrive.setVoltage(voltage);
	}

}
