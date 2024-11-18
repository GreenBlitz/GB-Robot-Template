package frc.robot.subsystems.swerve.module.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.extrainputs.DriveCouplingInputsAutoLogged;
import frc.robot.subsystems.swerve.module.records.DriveRequests;
import frc.robot.subsystems.swerve.module.records.DriveSignals;
import frc.robot.subsystems.swerve.module.records.EncoderSignals;
import frc.robot.subsystems.swerve.module.records.SteerRequests;
import frc.robot.subsystems.swerve.module.records.SteerSignals;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;


public class HardwareModule extends Module {

	@AutoLog
	public static class DriveCouplingInputs {

		public Rotation2d uncoupledVelocityAnglesPerSecond = new Rotation2d();
		public Rotation2d[] uncoupledPositions = new Rotation2d[0];

	}

	private final IAngleEncoder encoder;
	private final EncoderSignals encoderSignals;

	private final ControllableMotor steer;
	private final SteerRequests steerRequests;
	private final SteerSignals steerSignals;

	private final ControllableMotor drive;
	private final DriveRequests driveRequests;
	private final DriveSignals driveSignals;

	private final DriveCouplingInputsAutoLogged driveCouplingInputs;

	private Rotation2d startingSteerPosition;

	public HardwareModule(
		ModuleConstants constants,
		IAngleEncoder encoder,
		EncoderSignals encoderSignals,
		ControllableMotor steer,
		SteerRequests steerRequests,
		SteerSignals steerSignals,
		ControllableMotor drive,
		DriveRequests driveRequests,
		DriveSignals driveSignals
	) {
		super(constants);
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
		this.driveCouplingInputs = new DriveCouplingInputsAutoLogged();

		updateInputs();
		resetByEncoder();
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSteerSysIdConfigInfo() {
		return steer.getSysidConfigInfo();
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getDriveSysIdConfigInfo() {
		return drive.getSysidConfigInfo();
	}

	private void fixDriveInputsCoupling() {
		driveCouplingInputs.uncoupledVelocityAnglesPerSecond = ModuleUtils
			.uncoupleDriveAngle(driveSignals.velocity().getLatestValue(), steerSignals.velocity().getLatestValue(), constants.couplingRatio());

		driveCouplingInputs.uncoupledPositions = new Rotation2d[driveSignals.position().asArray().length];
		for (int i = 0; i < driveCouplingInputs.uncoupledPositions.length; i++) {
			Rotation2d steerDelta = Rotation2d
				.fromRotations(steerSignals.position().asArray()[i].getRotations() - startingSteerPosition.getRotations());
			driveCouplingInputs.uncoupledPositions[i] = ModuleUtils
				.uncoupleDriveAngle(driveSignals.position().asArray()[i], steerDelta, constants.couplingRatio());
		}
	}

	@Override
	public void updateInputs() {
		steer.updateSimulation();
		drive.updateSimulation();

		encoder.updateInputs(encoderSignals.position());
		steer.updateInputs(steerSignals.position(), steerSignals.velocity(), steerSignals.current(), steerSignals.voltage());
		drive.updateInputs(driveSignals.position(), driveSignals.velocity(), driveSignals.current(), driveSignals.voltage());

		fixDriveInputsCoupling();

		Logger.processInputs(constants.logPath() + "/Drive", driveCouplingInputs);
	}

	@Override
	public void setBrake(boolean brake) {
		steer.setBrake(brake);
		drive.setBrake(brake);
	}

	@Override
	public void resetByEncoder() {
		startingSteerPosition = encoderSignals.position().getLatestValue();
		steer.resetPosition(startingSteerPosition);
	}

	@Override
	public void setDriveVoltage(double voltage) {
		setClosedLoop(false);
		drive.applyRequest(driveRequests.voltage().withSetPoint(voltage));
	}

	@Override
	public void setSteerVoltage(double voltage) {
		steer.applyRequest(steerRequests.voltage().withSetPoint(voltage));
	}

	@Override
	public void setTargetSteerPosition(Rotation2d targetSteerPosition) {
		steer.applyRequest(steerRequests.position().withSetPoint(targetSteerPosition));
	}

	@Override
	public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
		setClosedLoop(true);
		Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(targetVelocityMetersPerSecond, constants.wheelDiameterMeters());
		Rotation2d coupledVelocityPerSecond = ModuleUtils
			.coupleDriveAngle(targetVelocityPerSecond, steerSignals.velocity().getLatestValue(), constants.couplingRatio());
		drive.applyRequest(driveRequests.velocity().withSetPoint(coupledVelocityPerSecond));
	}

	@Override
	public Rotation2d[] getDrivePositions() {
		return driveCouplingInputs.uncoupledPositions;
	}

	@Override
	public Rotation2d getDriveVelocitySeconds() {
		return driveCouplingInputs.uncoupledVelocityAnglesPerSecond;
	}

	@Override
	public Rotation2d[] getSteerPositions() {
		return steerSignals.position().asArray();
	}

	@Override
	public Rotation2d getSteerVelocitySeconds() {
		return steerSignals.velocity().getLatestValue();
	}

}
