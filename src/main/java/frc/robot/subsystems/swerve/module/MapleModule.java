package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.module.extrainputs.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.module.extrainputs.ModuleInputsAutoLogged;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;


public class MapleModule implements IModule {

	@AutoLog
	public static class ModuleIOInputs {

		public double drivePositionRad = 0.0;
		public double driveVelocityRadPerSec = 0.0;
		public double driveAppliedVolts = 0.0;
		public double[] driveCurrentAmps = new double[] {};

		public Rotation2d turnAbsolutePosition = new Rotation2d();
		public Rotation2d turnPosition = new Rotation2d();
		public double turnVelocityRadPerSec = 0.0;
		public double turnAppliedVolts = 0.0;
		public double[] turnCurrentAmps = new double[] {};

	}

	private final String logPath;
	private final SwerveModuleSimulation moduleSimulation;
	private final ModuleInputsAutoLogged moduleInputs;
	private final DriveInputsAutoLogged driveInputs;
	private final ModuleIOInputsAutoLogged moduleIOInputs;
	private SwerveModuleState targetState = new SwerveModuleState();
	private final double wheelDiameter;
	private final PIDController turnFeedback;


	public MapleModule(String logPath, SwerveModuleSimulation moduleSimulation, double wheelDiameter) {
		this.logPath = logPath;
		this.moduleSimulation = moduleSimulation;
		this.wheelDiameter = wheelDiameter;

		this.driveInputs = new DriveInputsAutoLogged();
		this.moduleInputs = new ModuleInputsAutoLogged();
		this.moduleIOInputs = new ModuleIOInputsAutoLogged();

		turnFeedback = new PIDController(7.0, 0.0, 0.0);
		turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
	}

	public double toDriveMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, wheelDiameter);
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSteerSysIdConfigInfo() {
		return new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), false);
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getDriveSysIdConfigInfo() {
		return new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), false);
	}

	@Override
	public void updateInputs() {
		moduleIOInputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPositionRad();
		moduleIOInputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeedRadPerSec();
		moduleIOInputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
		moduleIOInputs.driveCurrentAmps = new double[] {Math.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps())};

		moduleIOInputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
		moduleIOInputs.turnPosition = Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderPositionRad());
		moduleIOInputs.turnVelocityRadPerSec = moduleSimulation.getSteerRelativeEncoderSpeedRadPerSec();
		moduleIOInputs.turnAppliedVolts = moduleSimulation.getSteerMotorAppliedVolts();
		moduleIOInputs.turnCurrentAmps = new double[] {Math.abs(moduleSimulation.getSteerMotorSupplyCurrentAmps())};
		Logger.processInputs(logPath + "/moduleIO", moduleIOInputs);


		driveInputs.velocityMetersPerSecond = toDriveMeters(Rotation2d.fromRadians(moduleIOInputs.driveVelocityRadPerSec));
		driveInputs.positionsMeters = new double[] {toDriveMeters(Rotation2d.fromRadians(moduleIOInputs.drivePositionRad))};
		moduleInputs.isClosedLoop = false;
		moduleInputs.targetState = targetState;
		Logger.processInputs(logPath, moduleInputs);
		Logger.processInputs(logPath + "/drive", driveInputs);
	}

	@Override
	public void setBrake(boolean brake) {}

	@Override
	public void resetByEncoder() {}

	@Override
	public void stop() {
		moduleSimulation.requestDriveVoltageOut(0);
	}

	@Override
	public void setDriveVoltage(double voltage) {
		moduleSimulation.requestDriveVoltageOut(voltage);
	}

	@Override
	public void setSteerVoltage(double voltage) {
		moduleSimulation.requestSteerVoltageOut(voltage);
	}

	@Override
	public void pointSteer(Rotation2d steerTargetPosition, boolean optimize) {
		SwerveModuleState moduleState = new SwerveModuleState(0, steerTargetPosition);
		targetState.angle = optimize ? SwerveModuleState.optimize(moduleState, getSteerPosition()).angle : moduleState.angle;
		moveSteerToPosition(targetState.angle);
	}

	private void moveSteerToPosition(Rotation2d position) {
		moduleSimulation.requestSteerVoltageOut(turnFeedback.calculate(getSteerPosition().getRadians(), position.getRadians()));
	}

	@Override
	public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop) {
		this.targetState = SwerveModuleState.optimize(targetState, getSteerPosition());
		moveSteerToPosition(this.targetState.angle);
		setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle, isClosedLoop);
	}

	@Override
	public void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerPosition, boolean isClosedLoop) {
		double voltage = ModuleUtils.velocityToOpenLoopVoltage(
			targetVelocityMetersPerSecond,
			new Rotation2d(),
			0,
			Conversions.distanceToAngle(5, wheelDiameter),
			wheelDiameter,
			ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
		);
		setDriveVoltage(voltage);
	}

	@Override
	public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
		return new SwerveModulePosition(driveInputs.positionsMeters[0], moduleIOInputs.turnAbsolutePosition);
	}

	@Override
	public int getNumberOfOdometrySamples() {
		return 1;
	}

	@Override
	public SwerveModuleState getTargetState() {
		return targetState;
	}

	@Override
	public SwerveModuleState getCurrentState() {
		return new SwerveModuleState(driveInputs.velocityMetersPerSecond, getSteerPosition());
	}

	@Override
	public Rotation2d getDriveAngle() {
		return Rotation2d.fromRadians(moduleIOInputs.drivePositionRad);
	}

	@Override
	public double getDriveVelocityMetersPerSecond() {
		return driveInputs.velocityMetersPerSecond;
	}

	@Override
	public Rotation2d getSteerPosition() {
		return moduleIOInputs.turnAbsolutePosition;
	}

	@Override
	public boolean isAtTargetVelocity(double speedToleranceMetersPerSecond) {
		return false;
	}

	@Override
	public boolean isSteerAtTargetPosition(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband) {
		return false;
	}

	@Override
	public boolean isAtTargetState(Rotation2d steerTolerance, Rotation2d steerVelocityPerSecondDeadband, double speedToleranceMetersPerSecond) {
		return false;
	}

}
