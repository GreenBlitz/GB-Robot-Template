package frc.robot.subsystems.moduleAlon;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.AlonFX;
import frc.constants.AlonModuleConstants;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

public class ModuleAlon extends GBSubsystem {

	private final TalonFXMotor linear;
	private final TalonFXMotor steer;

	private final TalonFXConfiguration linearConfig;
	private final TalonFXConfiguration steerConfig;


	private final InputSignal<Rotation2d> linearPositionSignal;
	private final InputSignal<Rotation2d> linearVelocitySignal;
	private final InputSignal<Double> linearVoltageSignal;

	private final InputSignal<Rotation2d> steerPositionSignal;
	private final InputSignal<Rotation2d> steerVelocitySignal;
	private final InputSignal<Double> steerVoltageSignal;

	private final IAngleEncoder canCoder;
	private final String logPath;
	private final ModuleCommandsBuilder commandsBuilder;
	private State state = State.NORMAL;

	public void setState(State state) {
		this.state = state;
	}

	public State getState() {
		return state;
	}

	public static enum State {

		NORMAL(1),
		SLOW(0.1);

		public final double powerMultiplier;

		private State(double powerMultiplier) {
			this.powerMultiplier = powerMultiplier;
		}

	}

	public ModuleAlon(String logPath) {
		super(logPath);

		this.steer = new TalonFXMotor(
			logPath + "/steer",
			AlonModuleConstants.steerId,
			new SysIdRoutine.Config(),
			AlonFX.createSimulation(AlonModuleConstants.steerMomentOfInertiaKgM2, AlonModuleConstants.steerGearRatio)
		);
		this.linear = new TalonFXMotor(
			logPath + "/linear",
			AlonModuleConstants.linearId,
			new SysIdRoutine.Config(),
			AlonFX.createSimulation(AlonModuleConstants.linearMomentOfInertiaKgM2, AlonModuleConstants.linearGearRatio)
		);
		this.canCoder = EncoderFactory.createEncoder(logPath, ModuleUtil.ModulePosition.FRONT_LEFT);

		steerConfig = AlonFX.createConfig(AlonModuleConstants.steerGearRatio, AlonModuleConstants.steerKp);
		linearConfig = AlonFX.createConfig(AlonModuleConstants.linearGearRatio, AlonModuleConstants.linearKp);


		// if (RobotManager.isReal()){
		// steerConfig.Feedback.withFusedCANcoder(((CANCoderEncoder) canCoder).getDevice());
		// resetSteerWithCancoder();
		// }


		linearVelocitySignal = Phoenix6SignalBuilder.buildAngleSignal(
			linear.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			BusChain.ROBORIO
		);
		linearPositionSignal = Phoenix6SignalBuilder.buildLatencySignal(
			linear.getDevice().getPosition(),
			linear.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			BusChain.ROBORIO
		);
		linearVoltageSignal = Phoenix6SignalBuilder
			.buildDoubleSignal(linear.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);

		steerVelocitySignal = Phoenix6SignalBuilder.buildAngleSignal(
			steer.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			BusChain.ROBORIO
		);
		steerPositionSignal = Phoenix6SignalBuilder.buildLatencySignal(
			steer.getDevice().getPosition(),
			steer.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			BusChain.ROBORIO
		);
		steerVoltageSignal = Phoenix6SignalBuilder
			.buildDoubleSignal(steer.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);


		steer.applyConfiguration(steerConfig);
		linear.applyConfiguration(linearConfig);

		this.logPath = logPath;

		commandsBuilder = new ModuleCommandsBuilder(this, logPath + "/commandsBuilder");
		Command defaultCommand = new InstantCommand(() -> {
			stop();
		});
		defaultCommand.addRequirements(this);
		setDefaultCommand(defaultCommand);
	}

	public ModuleCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void invertLinear() {
		linear.applyConfiguration(AlonFX.invertMotorDirection(linearConfig));
	}

	public void invertSteer() {
		steer.applyConfiguration(AlonFX.invertMotorDirection(steerConfig));
	}

	public void steerToPosition(Rotation2d angle) {
		steer.applyRequest(Phoenix6RequestBuilder.buildVelocityPositionRequest(new PositionVoltage(angle.getMeasure()), 1, true));
	}

	public void linearPower(double power) {
		linear.setPower(power);
	}

	public Rotation2d getSteerPosition() {
		return steerPositionSignal.getLatestValue();
	}

	public Rotation2d getLinearPosition() {
		return linearPositionSignal.getLatestValue();
	}

	public Rotation2d getLinearVelocity() {
		return linearVelocitySignal.getLatestValue();
	}

	public void setLinearNeutral(boolean brake) {
		linear.setBrake(brake);
	}

	public void setSteerNeutral(boolean brake) {
		steer.setBrake(brake);
	}

	public void setSteerPosition(Rotation2d position) {
		steer.resetPosition(position);
	}

	public void setLinearPosition(Rotation2d position) {
		linear.resetPosition(position);
	}

	public void resetSteerWithCancoder() {
		steer.resetPosition(Rotation2d.fromRotations(((CANCoderEncoder) canCoder).getDevice().getPosition().getValue().in(Units.Rotation)));
	}

	public void logAll() {
		Logger.recordOutput(logPath + "/steerTarget", steer.getDevice().getClosedLoopReference().getValue());
		Logger.recordOutput(logPath + "/linearTarget", linear.getDevice().getClosedLoopReference().getValue());

		steer.updateInputs(steerPositionSignal, steerVelocitySignal, steerVoltageSignal);
		linear.updateInputs(linearPositionSignal, linearVelocitySignal, linearVoltageSignal);

		Translation2d direction = new Translation2d(linear.getDevice().getPosition().getValue().in(Units.Rotation), 0)
			.rotateBy(getSteerPosition());
		Logger.recordOutput(logPath + "/directionAngle", direction.getAngle());
	}

	public void stop() {
		linear.stop();
		steer.stop();
	}

	public void updateSimulation() {
		linear.updateSimulation();
		steer.updateSimulation();
	}

}
