package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
		public double[] driveCurrentAmps = {0};

		public Rotation2d turnAbsolutePosition = new Rotation2d();
		public Rotation2d turnPosition = new Rotation2d();
		public double turnVelocityRadPerSec = 0.0;
		public double turnAppliedVolts = 0.0;
		public double[] turnCurrentAmps = {0};

	}

	private final SwerveModuleSimulation moduleSimulation;
	private final ModuleIOInputsAutoLogged moduleIOInputs;
	private final PIDController steerPIDController;

	public MapleModule(ModuleConstants constants, SwerveModuleSimulation moduleSimulation) {
		super(constants);
		this.moduleSimulation = moduleSimulation;
		this.moduleIOInputs = new ModuleIOInputsAutoLogged();
		this.steerPIDController = new PIDController(7.0, 0.0, 0.0);
		steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
		setSteerVoltage(steerPIDController.calculate(getSteerLatestPosition().getRadians(), position.getRadians()));
	}

	@Override
	public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
		// TODO
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
	public Rotation2d[] getDrivePositions() {
		return new Rotation2d[] {Rotation2d.fromRadians(moduleIOInputs.drivePositionRad)};
	}

	@Override
	public Rotation2d getDriveVelocitySeconds() {
		return Rotation2d.fromRadians(moduleIOInputs.driveVelocityRadPerSec);
	}

	@Override
	public Rotation2d[] getSteerPositions() {
		return new Rotation2d[] {moduleIOInputs.turnAbsolutePosition};
	}

	@Override
	public Rotation2d getSteerVelocitySeconds() {
		return Rotation2d.fromRadians(moduleIOInputs.turnVelocityRadPerSec);
	}

}
