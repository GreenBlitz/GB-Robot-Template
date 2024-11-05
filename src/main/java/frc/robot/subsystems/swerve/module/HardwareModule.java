package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.angleencoder.IAngleEncoder;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.subsystems.swerve.module.extrainputs.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.module.stuffs.DriveStuff;
import frc.robot.subsystems.swerve.module.stuffs.EncoderStuff;
import frc.robot.subsystems.swerve.module.stuffs.SteerStuff;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class HardwareModule extends Module {

	private final IAngleEncoder encoder;
	private final EncoderStuff encoderStuff;

	private final ControllableMotor steer;
	private final IRequest<Rotation2d> steerPositionRequest;
	private final IRequest<Double> steerVoltageRequest;
	private final SteerStuff steerStuff;

	private final ControllableMotor drive;
	private final IRequest<Rotation2d> driveVelocityRequest;
	private final IRequest<Double> driveVoltageRequest;
	private final DriveStuff driveStuff;

	private final DriveInputsAutoLogged driveInputs;

	private Rotation2d startingSteerPosition;

	public HardwareModule(ModuleConstants constants, EncoderStuff encoderStuff, SteerStuff steerStuff, DriveStuff driveStuff) {
		super(constants);
		this.encoder = encoderStuff.encoder();
		this.encoderStuff = encoderStuff;

		this.steer = steerStuff.steer();
		this.steerVoltageRequest = steerStuff.voltageRequest();
		this.steerPositionRequest = steerStuff.positionRequest();
		this.steerStuff = steerStuff;

		this.drive = driveStuff.drive();
		this.driveVoltageRequest = steerStuff.voltageRequest();
		this.driveVelocityRequest = driveStuff.velocityRequest();
		this.driveStuff = driveStuff;

		this.targetState = new SwerveModuleState();
		this.startingSteerPosition = new Rotation2d();
		this.driveInputs = new DriveInputsAutoLogged();

		updateInputs();
		resetByEncoder();
	}

	public SysIdCalibrator.SysIdConfigInfo getSteerSysIdConfigInfo() {
		return steer.getSysidConfigInfo();
	}

	public SysIdCalibrator.SysIdConfigInfo getDriveSysIdConfigInfo() {
		return drive.getSysidConfigInfo();
	}

	private void fixDriveInputsCoupling() {
		driveInputs.uncoupledVelocityPerSecond = ModuleUtils.uncoupleDriveAngle(
			driveStuff.velocitySignal().getLatestValue(),
			steerStuff.velocitySignal().getLatestValue(),
			constants.couplingRatio()
		);

		driveInputs.uncoupledPositions = new Rotation2d[driveStuff.positionSignal().asArray().length];
		for (int i = 0; i < driveInputs.uncoupledPositions.length; i++) {
			Rotation2d steerDelta = Rotation2d
				.fromRotations(steerStuff.positionSignal().asArray()[i].getRotations() - startingSteerPosition.getRotations());
			driveInputs.uncoupledPositions[i] = ModuleUtils
				.uncoupleDriveAngle(driveStuff.positionSignal().asArray()[i], steerDelta, constants.couplingRatio());
		}
	}

	public void updateInputs() {
		encoder.updateInputs(encoderStuff.positionSignal());
		steer.updateInputs(steerStuff.positionSignal(), steerStuff.velocitySignal(), steerStuff.currentSignal(), steerStuff.voltageSignal());
		drive.updateInputs(driveStuff.positionSignal(), driveStuff.velocitySignal(), driveStuff.currentSignal(), driveStuff.voltageSignal());

		fixDriveInputsCoupling();

		driveInputs.velocityMetersPerSecond = toDriveMeters(driveInputs.uncoupledVelocityPerSecond);
		driveInputs.positionsMeters = Arrays.stream(driveInputs.uncoupledPositions).mapToDouble(this::toDriveMeters).toArray();

		Logger.processInputs(driveStuff.logPath(), driveInputs);
	}

	public void setBrake(boolean brake) {
		steer.setBrake(brake);
		drive.setBrake(brake);
	}

	public void resetByEncoder() {
		startingSteerPosition = encoderStuff.positionSignal().getLatestValue();
		steer.resetPosition(startingSteerPosition);
	}


	public void setDriveVoltage(double voltage) {
		setClosedLoop(false);
		drive.applyDoubleRequest(driveVoltageRequest.withSetPoint(voltage));
	}

	public void setSteerVoltage(double voltage) {
		steer.applyDoubleRequest(steerVoltageRequest.withSetPoint(voltage));
	}


	@Override
	public void setTargetSteerPosition(Rotation2d position) {
		steer.applyAngleRequest(steerPositionRequest.withSetPoint(position));
	}

	@Override
	public void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerPosition, boolean isClosedLoop) {
		targetVelocityMetersPerSecond = ModuleUtils.reduceSkew(targetVelocityMetersPerSecond, targetSteerPosition, getSteerPosition());

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
			.coupleDriveAngle(targetVelocityPerSecond, getSteerVelocitySeconds(), constants.couplingRatio());
		drive.applyAngleRequest(driveVelocityRequest.withSetPoint(coupledVelocityPerSecond));
	}

	public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
		double voltage = ModuleUtils.velocityToOpenLoopVoltage(
			targetVelocityMetersPerSecond,
			steerStuff.velocitySignal().getLatestValue(),
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
	@Override
	public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
		return new SwerveModulePosition(
			driveInputs.positionsMeters[odometryUpdateIndex],
			steerStuff.positionSignal().asArray()[odometryUpdateIndex]
		);
	}

	@Override
	public int getNumberOfOdometrySamples() {
		return Math.min(driveInputs.positionsMeters.length, steerStuff.positionSignal().asArray().length);
	}

	@Override
	public Rotation2d getDriveAngle() {
		return driveInputs.uncoupledPositions[driveInputs.uncoupledPositions.length - 1];
	}

	@Override
	public Rotation2d getDriveVelocitySeconds() {
		return driveInputs.uncoupledVelocityPerSecond;
	}

	@Override
	public Rotation2d getSteerPosition() {
		return steerStuff.positionSignal().getLatestValue();
	}

	@Override
	public Rotation2d getSteerVelocitySeconds() {
		return steerStuff.velocitySignal().getLatestValue();
	}

}
