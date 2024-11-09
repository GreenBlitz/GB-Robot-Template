package frc.robot.subsystems.swerve.module.maple;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class MapleModule extends Module {

	@AutoLog
	public static class ModuleIOInputs {

		public Rotation2d drivePosition = new Rotation2d();
		public Rotation2d driveVelocityPositionPerSecond = new Rotation2d();
		public double driveAppliedVolts = 0.0;
		public double[] driveCurrentAmps = {0};

		public Rotation2d turnAbsolutePosition = new Rotation2d();
		public Rotation2d turnPosition = new Rotation2d();
		public Rotation2d turnVelocityPositionPerSecond = new Rotation2d();
		public double turnAppliedVolts = 0.0;
		public double[] turnCurrentAmps = {0};

	}

	private final SwerveModuleSimulation moduleSimulation;
	private final ModuleIOInputsAutoLogged moduleIOInputs;
	private final PIDController steerPIDControllerRadians;
	private final PIDController drivePIDControllerRadiansPerSecond;
	private final SimpleMotorFeedforward driveFeedForwardRadiansPerSecond;

	public MapleModule(ModuleConstants constants, MapleModuleConstants mapleModuleConstants, SwerveModuleSimulation moduleSimulation) {
		super(constants);
		this.moduleSimulation = moduleSimulation;
		this.moduleIOInputs = new ModuleIOInputsAutoLogged();
		this.steerPIDControllerRadians = mapleModuleConstants.steerPIDControllerRadians();
		this.drivePIDControllerRadiansPerSecond = mapleModuleConstants.drivePIDControllerRadiansPerSecond();
		this.driveFeedForwardRadiansPerSecond = mapleModuleConstants.driveFeedForwardRadiansPerSecond();
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
		moduleIOInputs.drivePosition = Rotation2d.fromRadians(moduleSimulation.getDriveWheelFinalPositionRad());
		moduleIOInputs.driveVelocityPositionPerSecond = Rotation2d.fromRadians(moduleSimulation.getDriveWheelFinalSpeedRadPerSec());
		moduleIOInputs.driveAppliedVolts = moduleSimulation.getDriveMotorAppliedVolts();
		moduleIOInputs.driveCurrentAmps = new double[] {Math.abs(moduleSimulation.getDriveMotorSupplyCurrentAmps())};

		moduleIOInputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
		moduleIOInputs.turnPosition = Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderPositionRad());
		moduleIOInputs.turnVelocityPositionPerSecond = Rotation2d.fromRadians(moduleSimulation.getSteerRelativeEncoderSpeedRadPerSec());
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
	public void setTargetSteerPosition(Rotation2d targetSteerPosition) {
		setSteerVoltage(steerPIDControllerRadians.calculate(getSteerLatestPosition().getRadians(), targetSteerPosition.getRadians()));
	}

	@Override
	public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
		Rotation2d targetVelocityRadiansPerSecond = fromDriveMeters(targetVelocityMetersPerSecond);
		double voltage = driveFeedForwardRadiansPerSecond.calculate(targetVelocityRadiansPerSecond.getRadians())
			+ drivePIDControllerRadiansPerSecond.calculate(getDriveVelocitySeconds().getRadians(), targetVelocityRadiansPerSecond.getRadians());
		setDriveVoltage(voltage);
	}

	@Override
	public Rotation2d[] getDrivePositions() {
		return new Rotation2d[] {moduleIOInputs.drivePosition};
	}

	@Override
	public Rotation2d getDriveVelocitySeconds() {
		return moduleIOInputs.driveVelocityPositionPerSecond;
	}

	@Override
	public Rotation2d[] getSteerPositions() {
		return new Rotation2d[] {moduleIOInputs.turnAbsolutePosition};
	}

	@Override
	public Rotation2d getSteerVelocitySeconds() {
		return moduleIOInputs.turnVelocityPositionPerSecond;
	}

}
