package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.module.extrainputs.DriveInputsAutoLogged;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;


public class MapleModule extends Module {

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

	private final SwerveModuleSimulation moduleSimulation;
	private final DriveInputsAutoLogged driveInputs;
	private final ModuleIOInputsAutoLogged moduleIOInputs;
	private final PIDController turnFeedback;


	public MapleModule(ModuleConstants constants, SwerveModuleSimulation moduleSimulation) {
		super(constants);
		this.moduleSimulation = moduleSimulation;

		this.driveInputs = new DriveInputsAutoLogged();
		this.moduleIOInputs = new ModuleIOInputsAutoLogged();

		turnFeedback = new PIDController(7.0, 0.0, 0.0);
		turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
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
		Logger.processInputs(constants.logPath() + "/moduleIO", moduleIOInputs);


		driveInputs.velocityMetersPerSecond = toDriveMeters(Rotation2d.fromRadians(moduleIOInputs.driveVelocityRadPerSec));
		driveInputs.positionsMeters = new double[] {toDriveMeters(Rotation2d.fromRadians(moduleIOInputs.drivePositionRad))};
		Logger.processInputs(constants.logPath() + "/drive", driveInputs);
	}

	@Override
	public void setBrake(boolean brake) {}

	@Override
	public void resetByEncoder() {}

	@Override
	public void setDriveVoltage(double voltage) {
		setClosedLoop(false);
		moduleSimulation.requestDriveVoltageOut(voltage);
	}

	@Override
	public void setSteerVoltage(double voltage) {
		moduleSimulation.requestSteerVoltageOut(voltage);
	}


	@Override
	public void setTargetSteerPosition(Rotation2d position) {
		setSteerVoltage(turnFeedback.calculate(getSteerPosition().getRadians(), position.getRadians()));
	}

	@Override
	public void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerPosition, boolean isClosedLoop) {
		double voltage = ModuleUtils.velocityToOpenLoopVoltage(
			targetVelocityMetersPerSecond,
			new Rotation2d(),
			constants.couplingRatio(),
			constants.velocityAt12VoltsPerSecond(),
			constants.wheelDiameterMeters(),
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
	public Rotation2d getDriveAngle() {
		return Rotation2d.fromRadians(moduleIOInputs.drivePositionRad);
	}

	@Override
	public Rotation2d getDriveVelocitySeconds() {
		return Rotation2d.fromRadians(moduleIOInputs.driveVelocityRadPerSec);
	}

	@Override
	public Rotation2d getSteerPosition() {
		return moduleIOInputs.turnAbsolutePosition;
	}

	@Override
	public Rotation2d getSteerVelocitySeconds() {
		return Rotation2d.fromRadians(moduleIOInputs.turnVelocityRadPerSec);
	}

}
