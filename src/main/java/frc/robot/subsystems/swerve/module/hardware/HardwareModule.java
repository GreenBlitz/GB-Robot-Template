package frc.robot.subsystems.swerve.module.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.hardware.angleencoder.IAngleEncoder;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.stuffs.DriveStuff;
import frc.robot.subsystems.swerve.module.stuffs.EncoderStuff;
import frc.robot.subsystems.swerve.module.stuffs.SteerStuff;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class HardwareModule extends Module {

	@AutoLog
	public static class CouplingInputs {

		public Rotation2d uncoupledVelocityPerSecond = new Rotation2d();
		public Rotation2d[] uncoupledPositions = new Rotation2d[0];

	}

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

	private final CouplingInputsAutoLogged couplingInputs;

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
		this.couplingInputs = new CouplingInputsAutoLogged();

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
		couplingInputs.uncoupledVelocityPerSecond = ModuleUtils.uncoupleDriveAngle(
			driveStuff.velocitySignal().getLatestValue(),
			steerStuff.velocitySignal().getLatestValue(),
			constants.couplingRatio()
		);

		couplingInputs.uncoupledPositions = new Rotation2d[driveStuff.positionSignal().asArray().length];
		for (int i = 0; i < couplingInputs.uncoupledPositions.length; i++) {
			Rotation2d steerDelta = Rotation2d
				.fromRotations(steerStuff.positionSignal().asArray()[i].getRotations() - startingSteerPosition.getRotations());
			couplingInputs.uncoupledPositions[i] = ModuleUtils
				.uncoupleDriveAngle(driveStuff.positionSignal().asArray()[i], steerDelta, constants.couplingRatio());
		}
	}

	@Override
	public void updateInputs() {
		encoder.updateInputs(encoderStuff.positionSignal());
		steer.updateInputs(steerStuff.positionSignal(), steerStuff.velocitySignal(), steerStuff.currentSignal(), steerStuff.voltageSignal());
		drive.updateInputs(driveStuff.positionSignal(), driveStuff.velocitySignal(), driveStuff.currentSignal(), driveStuff.voltageSignal());

		fixDriveInputsCoupling();

		Logger.processInputs(driveStuff.logPath(), couplingInputs);
	}

	@Override
	public void setBrake(boolean brake) {
		steer.setBrake(brake);
		drive.setBrake(brake);
	}

	@Override
	public void resetByEncoder() {
		startingSteerPosition = encoderStuff.positionSignal().getLatestValue();
		steer.resetPosition(startingSteerPosition);
	}

	@Override
	public void setDriveVoltage(double voltage) {
		setClosedLoop(false);
		drive.applyRequest(driveVoltageRequest.withSetPoint(voltage));
	}

	@Override
	public void setSteerVoltage(double voltage) {
		steer.applyRequest(steerVoltageRequest.withSetPoint(voltage));
	}

	@Override
	public void setTargetSteerPosition(Rotation2d targetSteerPosition) {
		steer.applyRequest(steerPositionRequest.withSetPoint(targetSteerPosition));
	}

	@Override
	public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
		setClosedLoop(true);
		Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(targetVelocityMetersPerSecond, constants.wheelDiameterMeters());
		Rotation2d coupledVelocityPerSecond = ModuleUtils
			.coupleDriveAngle(targetVelocityPerSecond, getSteerVelocitySeconds(), constants.couplingRatio());
		drive.applyRequest(driveVelocityRequest.withSetPoint(coupledVelocityPerSecond));
	}

	@Override
	public Rotation2d[] getDrivePositions() {
		return couplingInputs.uncoupledPositions;
	}

	@Override
	public Rotation2d getDriveVelocitySeconds() {
		return couplingInputs.uncoupledVelocityPerSecond;
	}

	@Override
	public Rotation2d[] getSteerPositions() {
		return steerStuff.positionSignal().asArray();
	}

	@Override
	public Rotation2d getSteerVelocitySeconds() {
		return steerStuff.velocitySignal().getLatestValue();
	}

}
