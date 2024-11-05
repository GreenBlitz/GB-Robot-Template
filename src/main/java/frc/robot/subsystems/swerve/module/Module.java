package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.module.extrainputs.ModuleInputsAutoLogged;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

public abstract class Module {

	protected final ModuleConstants constants;
	private final ModuleInputsAutoLogged moduleInputs;
	private boolean isClosedLoop;
	protected SwerveModuleState targetState;

	public Module(ModuleConstants constants) {
		this.constants = constants;
		this.isClosedLoop = ModuleConstants.DEFAULT_IS_CLOSE_LOOP;
		this.moduleInputs = new ModuleInputsAutoLogged();
		this.targetState = new SwerveModuleState();
	}

	public double toDriveMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, constants.wheelDiameterMeters());
	}

	public void setClosedLoop(boolean closedLoop) {
		isClosedLoop = closedLoop;
	}

	public final void updateStatus() {
		updateInputs();
		moduleInputs.isClosedLoop = isClosedLoop;
		moduleInputs.targetState = targetState;
		Logger.processInputs(constants.logPath(), moduleInputs);
	}


	public void stop() {
		targetState = new SwerveModuleState(0, getSteerPosition());
		setSteerVoltage(0);
		setDriveVoltage(0);
	}

	public void pointSteer(Rotation2d steerTargetPosition, boolean optimize) {
		SwerveModuleState moduleState = new SwerveModuleState(0, steerTargetPosition);
		targetState.angle = optimize ? SwerveModuleState.optimize(moduleState, getSteerPosition()).angle : moduleState.angle;
		setTargetSteerPosition(targetState.angle);
	}

	public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop) {
		this.targetState = SwerveModuleState.optimize(targetState, getSteerPosition());
		setTargetSteerPosition(this.targetState.angle);
		setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle, isClosedLoop);
	}


	public SwerveModuleState getTargetState() {
		return targetState;
	}

	public SwerveModuleState getCurrentState() {
		return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getSteerPosition());
	}

	public double getDriveVelocityMetersPerSecond() {
		return toDriveMeters(getDriveVelocitySeconds());
	}


	//@formatter:off
	public boolean isAtTargetVelocity(double speedToleranceMetersPerSecond) {
		return MathUtil.isNear(
			getTargetState().speedMetersPerSecond,
			getDriveVelocityMetersPerSecond(),
			speedToleranceMetersPerSecond
		);
	}
	//@formatter:on

	public boolean isSteerAtTargetPosition(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband) {
		boolean isStopping = getSteerVelocitySeconds().getRadians() <= steerVelocityPerSecondDeadband.getRadians();
		if (!isStopping) {
			return false;
		}
		boolean isAtSteerPosition = MathUtil.isNear(
			MathUtil.angleModulus(getTargetState().angle.getRadians()),
			MathUtil.angleModulus(getSteerPosition().getRadians()),
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

	public abstract void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerPosition, boolean isClosedLoop);

	public abstract void setTargetSteerPosition(Rotation2d position);

	/**
	 * The odometry thread can update itself faster than the main code loop (which is 50 hertz). Instead of using the latest odometry update, the
	 * accumulated odometry positions since the last loop to get a more accurate position.
	 *
	 * @param odometryUpdateIndex the index of the odometry update
	 * @return the position of the module at the given odometry update index
	 */
	public abstract SwerveModulePosition getOdometryPosition(int odometryUpdateIndex);

	public abstract int getNumberOfOdometrySamples();

	public abstract Rotation2d getDriveAngle();

	public abstract Rotation2d getDriveVelocitySeconds();

	public abstract Rotation2d getSteerPosition();

	public abstract Rotation2d getSteerVelocitySeconds();

	public abstract void updateInputs();

}
