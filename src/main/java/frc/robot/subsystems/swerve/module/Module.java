package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.constants.MathConstants;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.swerve.module.extrainputs.DriveCouplingInputsAutoLogged;
import frc.robot.subsystems.swerve.module.extrainputs.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.module.extrainputs.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.module.records.DriveRequests;
import frc.robot.subsystems.swerve.module.records.DriveSignals;
import frc.robot.subsystems.swerve.module.records.EncoderSignals;
import frc.robot.subsystems.swerve.module.records.ModuleSpecificConstants;
import frc.robot.subsystems.swerve.module.records.SteerRequests;
import frc.robot.subsystems.swerve.module.records.SteerSignals;
import frc.utils.Conversions;
import frc.utils.math.ToleranceMath;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Module {

	private final ModuleSpecificConstants constants;

	private final IAngleEncoder encoder;
	private final EncoderSignals encoderSignals;

	private final ControllableMotor steer;
	private final SteerRequests steerRequests;
	private final SteerSignals steerSignals;

	private final ControllableMotor drive;
	private final DriveRequests driveRequests;
	private final DriveSignals driveSignals;

	private final ModuleInputsAutoLogged moduleInputs;
	private final DriveInputsAutoLogged driveInputs;
	private final DriveCouplingInputsAutoLogged driveCouplingInputs;

	private SwerveModuleState targetState;
	private Rotation2d startingSteerPosition;
	private boolean isClosedLoop;

	public Module(
		ModuleSpecificConstants constants,
		IAngleEncoder encoder,
		EncoderSignals encoderSignals,
		ControllableMotor steer,
		SteerRequests steerRequests,
		SteerSignals steerSignals,
		ControllableMotor drive,
		DriveRequests driveRequests,
		DriveSignals driveSignals
	) {
		this.constants = constants;

		this.encoder = encoder;
		this.encoderSignals = encoderSignals;

		this.steer = steer;
		this.steerRequests = steerRequests;
		this.steerSignals = steerSignals;

		this.drive = drive;
		this.driveRequests = driveRequests;
		this.driveSignals = driveSignals;

		this.targetState = new SwerveModuleState();
		this.startingSteerPosition = new Rotation2d();
		this.isClosedLoop = ModuleConstants.DEFAULT_IS_CLOSE_LOOP;
		this.moduleInputs = new ModuleInputsAutoLogged();
		this.driveInputs = new DriveInputsAutoLogged();
		this.driveCouplingInputs = new DriveCouplingInputsAutoLogged();

		updateInputs();
		resetSteerByEncoder();
	}

	public SysIdCalibrator.SysIdConfigInfo getSteerSysIdConfigInfo() {
		return steer.getSysidConfigInfo();
	}

	public SysIdCalibrator.SysIdConfigInfo getDriveSysIdConfigInfo() {
		return drive.getSysidConfigInfo();
	}

	public Translation2d getPositionFromCenterMeters() {
		return constants.positionFromCenterMeters();
	}

	public double toDriveMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, constants.wheelDiameterMeters());
	}


	private void fixDriveInputsCoupling() {
		driveCouplingInputs.uncoupledVelocityAnglesPerSecond = ModuleUtil
			.uncoupleDriveAngle(driveSignals.velocity().getLatestValue(), steerSignals.velocity().getLatestValue(), constants.couplingRatio());

		driveCouplingInputs.uncoupledPositions = new Rotation2d[driveSignals.position().asArray().length];
		for (int i = 0; i < driveCouplingInputs.uncoupledPositions.length; i++) {
			Rotation2d steerDelta = Rotation2d
				.fromRotations(steerSignals.position().asArray()[i].getRotations() - startingSteerPosition.getRotations());
			driveCouplingInputs.uncoupledPositions[i] = ModuleUtil
				.uncoupleDriveAngle(driveSignals.position().asArray()[i], steerDelta, constants.couplingRatio());
		}
	}

	public void updateInputs() {
		steer.updateSimulation();
		drive.updateSimulation();

		encoder.updateInputs(encoderSignals.position());
		steer.updateInputs(steerSignals.position(), steerSignals.velocity(), steerSignals.current(), steerSignals.voltage());
		drive.updateInputs(driveSignals.position(), driveSignals.velocity(), driveSignals.current(), driveSignals.voltage());

		fixDriveInputsCoupling();

		driveInputs.velocityMetersPerSecond = toDriveMeters(driveCouplingInputs.uncoupledVelocityAnglesPerSecond);
		driveInputs.positionsMeters = Arrays.stream(driveCouplingInputs.uncoupledPositions).mapToDouble(this::toDriveMeters).toArray();

		moduleInputs.isClosedLoop = isClosedLoop;
		moduleInputs.targetState = targetState;

		Logger.processInputs(constants.logPath(), moduleInputs);
		Logger.processInputs(constants.logPath() + "/Drive", driveInputs);
		Logger.processInputs(constants.logPath() + "/Drive", driveCouplingInputs);
	}


	public void setClosedLoop(boolean closedLoop) {
		isClosedLoop = closedLoop;
	}

	public void setBrake(boolean brake) {
		steer.setBrake(brake);
		drive.setBrake(brake);
	}

	public void resetSteerByEncoder() {
		startingSteerPosition = encoderSignals.position().getLatestValue();
		steer.resetPosition(startingSteerPosition);
	}


	public void stop() {
		targetState = new SwerveModuleState(0, getSteerPosition());
		moduleInputs.controlMode = ModuleUtil.ControlMode.TARGET_STATE.toLog();
		steer.stop();
		drive.stop();
	}

	public void setDriveCurrent(double current) {
		drive.applyRequest(driveRequests.torqueCurrent().withSetPoint(current));
	}

	public void setDriveVoltage(double voltage) {
		setDriveVoltage(voltage, false);
	}

	private void setDriveVoltage(double voltage, boolean usingTargetState) {
		setClosedLoop(false);
		if (!usingTargetState) {
			moduleInputs.controlMode = ModuleUtil.ControlMode.CUSTOM.toLog();
		}
		drive.applyRequest(driveRequests.voltage().withSetPoint(voltage));
	}

	public void setSteerVoltage(double voltage) {
		moduleInputs.controlMode = ModuleUtil.ControlMode.CUSTOM.toLog();
		steer.applyRequest(steerRequests.voltage().withSetPoint(voltage));
	}


	public void pointToCenter() {
		double angleRadians = Math.atan(constants.positionFromCenterMeters().getY() / constants.positionFromCenterMeters().getX());
		pointSteer(Rotation2d.fromRadians(angleRadians), true);
	}

	public void pointInCircle() {
		double angleRadians = Math.atan(constants.positionFromCenterMeters().getY() / constants.positionFromCenterMeters().getX());
		angleRadians -= MathConstants.QUARTER_CIRCLE.getRadians();
		pointSteer(Rotation2d.fromRadians(angleRadians), true);
	}

	public void pointSteer(Rotation2d steerTargetPosition, boolean optimize) {
		SwerveModuleState moduleState = new SwerveModuleState(0, steerTargetPosition);
		if (optimize) {
			moduleState.optimize(getSteerPosition());
		}
		targetState.angle = moduleState.angle;
		moduleInputs.controlMode = ModuleUtil.ControlMode.TARGET_STATE.toLog();
		setTargetSteerPosition(targetState.angle);
	}


	public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop) {
		targetState.optimize(getSteerPosition());
		targetState.cosineScale(getSteerPosition());

		this.targetState = targetState;
		moduleInputs.controlMode = ModuleUtil.ControlMode.TARGET_STATE.toLog();

		setTargetSteerPosition(this.targetState.angle);
		setTargetVelocity(this.targetState.speedMetersPerSecond, isClosedLoop);
	}

	private void setTargetSteerPosition(Rotation2d targetSteerPosition) {
		steer.applyRequest(steerRequests.position().withSetPoint(targetSteerPosition));
	}

	public void setTargetVelocity(double targetVelocityMetersPerSecond, boolean isClosedLoop) {
		if (isClosedLoop) {
			setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
		} else {
			setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
		}
	}

	public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
		setClosedLoop(true);
		Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(targetVelocityMetersPerSecond, constants.wheelDiameterMeters());
		Rotation2d coupledVelocityPerSecond = ModuleUtil
			.coupleDriveAngle(targetVelocityPerSecond, steerSignals.velocity().getLatestValue(), constants.couplingRatio());
		drive.applyRequest(driveRequests.velocity().withSetPoint(coupledVelocityPerSecond));
	}

	public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
		double voltage = ModuleUtil.velocityToOpenLoopVoltage(
			targetVelocityMetersPerSecond,
			steerSignals.velocity().getLatestValue(),
			constants.couplingRatio(),
			constants.velocityAt12VoltsPerSecond(),
			constants.wheelDiameterMeters(),
			ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
		);
		setDriveVoltage(voltage, true);
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
			driveInputs.positionsMeters[odometryUpdateIndex],
			steerSignals.position().asArray()[odometryUpdateIndex]
		);
	}

	public int getNumberOfOdometrySamples() {
		return Math.min(driveInputs.positionsMeters.length, steerSignals.position().asArray().length);
	}

	public SwerveModuleState getTargetState() {
		return targetState;
	}

	public SwerveModuleState getCurrentState() {
		return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getSteerPosition());
	}

	public Rotation2d getDrivePosition() {
		return driveCouplingInputs.uncoupledPositions[driveCouplingInputs.uncoupledPositions.length - 1];
	}

	public double getDriveVelocityMetersPerSecond() {
		return driveInputs.velocityMetersPerSecond;
	}

	public Rotation2d getSteerPosition() {
		return steerSignals.position().getLatestValue();
	}


	public boolean isAtTargetVelocity(double speedToleranceMetersPerSecond) {
		return MathUtil.isNear(getTargetState().speedMetersPerSecond, getDriveVelocityMetersPerSecond(), speedToleranceMetersPerSecond);
	}

	public boolean isSteerAtTargetPosition(Rotation2d steerPositionTolerance, Rotation2d steerVelocityPerSecondDeadband) {
		boolean isStopping = steerSignals.velocity().getLatestValue().getRadians() <= steerVelocityPerSecondDeadband.getRadians();
		if (!isStopping) {
			return false;
		}
		boolean isAtSteerPosition = ToleranceMath.isNearWrapped(getTargetState().angle, getSteerPosition(), steerPositionTolerance);
		return isAtSteerPosition;
	}

	public boolean isAtTargetState(
		Rotation2d steerPositionTolerance,
		Rotation2d steerVelocityPerSecondDeadband,
		double speedToleranceMetersPerSecond
	) {
		return isSteerAtTargetPosition(steerPositionTolerance, steerVelocityPerSecondDeadband)
			&& isAtTargetVelocity(speedToleranceMetersPerSecond);
	}

}
