package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.module.extrainputs.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.module.extrainputs.ModuleInputsAutoLogged;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public abstract class Module {

	protected final ModuleConstants constants;
	private final ModuleInputsAutoLogged moduleInputs;
	private final DriveInputsAutoLogged driveInputs;
	private boolean isClosedLoop;
	protected SwerveModuleState targetState;

	public Module(ModuleConstants constants) {
		this.constants = constants;
		this.isClosedLoop = ModuleConstants.DEFAULT_IS_CLOSE_LOOP;
		this.moduleInputs = new ModuleInputsAutoLogged();
		this.driveInputs = new DriveInputsAutoLogged();
		this.targetState = new SwerveModuleState();
	}

	public double toDriveMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, constants.wheelDiameterMeters());
	}

	public Rotation2d fromDriveMeters(double distance) {
		return Conversions.distanceToAngle(distance, constants.wheelDiameterMeters());
	}

	public void setClosedLoop(boolean closedLoop) {
		isClosedLoop = closedLoop;
	}

	public final void updateStatus() {
		updateInputs();

		driveInputs.velocityMetersPerSecond = toDriveMeters(getDriveVelocitySeconds());
		driveInputs.positionsMeters = Arrays.stream(getDrivePositions()).mapToDouble(this::toDriveMeters).toArray();
		Logger.processInputs(constants.logPath() + "/drive", driveInputs);

		moduleInputs.isClosedLoop = isClosedLoop;
		moduleInputs.targetState = targetState;
		Logger.processInputs(constants.logPath(), moduleInputs);
	}

	public void stop() {
		targetState = new SwerveModuleState(0, getSteerLatestPosition());
		setSteerVoltage(0);
		setDriveVoltage(0);
	}

	public void pointSteer(Rotation2d steerTargetPosition, boolean optimize) {
		SwerveModuleState moduleState = new SwerveModuleState(0, steerTargetPosition);
		targetState.angle = optimize ? SwerveModuleState.optimize(moduleState, getSteerLatestPosition()).angle : moduleState.angle;
		setTargetSteerPosition(targetState.angle);
	}

	public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop) {
		this.targetState = SwerveModuleState.optimize(targetState, getSteerLatestPosition());
		setTargetSteerPosition(this.targetState.angle);
		setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle, isClosedLoop);
	}

	public void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerPosition, boolean isClosedLoop) {
		targetVelocityMetersPerSecond = ModuleUtils.reduceSkew(targetVelocityMetersPerSecond, targetSteerPosition, getSteerLatestPosition());
		setClosedLoop(isClosedLoop);

		if (isClosedLoop) {
			setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
		} else {
			setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
		}
	}

	public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
		double voltage = ModuleUtils.velocityToOpenLoopVoltage(
			targetVelocityMetersPerSecond,
			getSteerVelocitySeconds(),
			constants.couplingRatio(),
			constants.velocityAt12VoltsPerSecond(),
			constants.wheelDiameterMeters(),
			ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
		);
		setDriveVoltage(voltage);
	}

	/**
	 * The odometry thread can update itself faster than the main code loop (which is 50 hertz). Instead of using the latest odometry update, the
	 * accumulated odometry positions since the last loop to get a more accurate position.
	 *
	 * @param odometryUpdateIndex the index of the odometry update
	 * @return the position of the module at the given odometry update index
	 */
	public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
		return new SwerveModulePosition(driveInputs.positionsMeters[odometryUpdateIndex], getSteerPositions()[odometryUpdateIndex]);
	}

	public int getNumberOfOdometrySamples() {
		return Math.min(getDrivePositions().length, getSteerPositions().length);
	}

	public SwerveModuleState getTargetState() {
		return targetState;
	}

	public SwerveModuleState getCurrentState() {
		return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getSteerLatestPosition());
	}

	public Rotation2d getSteerLatestPosition() {
		Rotation2d[] steerPositions = getSteerPositions();
		return steerPositions[steerPositions.length - 1];
	}

	public Rotation2d getDriveLatestPosition() {
		Rotation2d[] drivePositions = getDrivePositions();
		return drivePositions[drivePositions.length - 1];
	}

	public double getDriveVelocityMetersPerSecond() {
		return toDriveMeters(getDriveVelocitySeconds());
	}


	public boolean isAtTargetVelocity(double speedToleranceMetersPerSecond) {
		return MathUtil.isNear(getTargetState().speedMetersPerSecond, getDriveVelocityMetersPerSecond(), speedToleranceMetersPerSecond);
	}

	public boolean isSteerAtTargetPosition(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband) {
		boolean isStopping = getSteerVelocitySeconds().getRadians() <= steerVelocityPerSecondDeadband.getRadians();
		if (!isStopping) {
			return false;
		}
		boolean isAtSteerPosition = MathUtil.isNear(
			MathUtil.angleModulus(getTargetState().angle.getRadians()),
			MathUtil.angleModulus(getSteerLatestPosition().getRadians()),
			steerTolerance.getRadians()
		);
		return isAtSteerPosition;
	}

	public boolean isAtTargetState(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband, double speedToleranceMetersPerSecond) {
		return isSteerAtTargetPosition(steerTolerance, steerVelocityPerSecondDeadband) && isAtTargetVelocity(speedToleranceMetersPerSecond);
	}


	public abstract SysIdCalibrator.SysIdConfigInfo getSteerSysIdConfigInfo();

	public abstract SysIdCalibrator.SysIdConfigInfo getDriveSysIdConfigInfo();

	public abstract void setBrake(boolean brake);

	public abstract void resetByEncoder();

	public abstract void setDriveVoltage(double voltage);

	public abstract void setSteerVoltage(double voltage);

	public abstract void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond);

	public abstract void setTargetSteerPosition(Rotation2d targetSteerPosition);

	public abstract Rotation2d[] getDrivePositions();

	public abstract Rotation2d getDriveVelocitySeconds();

	public abstract Rotation2d[] getSteerPositions();

	public abstract Rotation2d getSteerVelocitySeconds();

	public abstract void updateInputs();

}
