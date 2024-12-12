package frc.robot.subsystems.pivot.factory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.motor.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.PivotStuff;
import frc.utils.AngleUnit;

import static edu.wpi.first.units.Units.Second;

public class RealPivotConstants {

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Units.Volts.of(1.0).per(Second),
			Units.Volts.of(7.0),
			Units.Seconds.of(10.0),
			(state) -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateMotorConfig() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.Slot0.kP = 230;
		configuration.Slot0.kD = 10;
		configuration.Slot0.kV = 18.57;
		configuration.Slot0.kS = 0.26;
		configuration.Slot0.kA = 0.7702;
		configuration.Feedback.SensorToMechanismRatio = PivotConstants.GEAR_RATIO;

		configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.FORWARD_ANGLE_LIMIT.getRotations();
		configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.BACKWARD_ANGLE_LIMIT.getRotations();

		configuration.CurrentLimits.StatorCurrentLimitEnable = true;
		configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
		configuration.CurrentLimits.StatorCurrentLimit = 40;
		configuration.CurrentLimits.SupplyCurrentLimit = 40;
		configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return configuration;
	}

	protected static PivotStuff generatePivotStuff(String logPath) {
		Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0).withEnableFOC(true));

		TalonFXMotor pivot = new TalonFXMotor(logPath, IDs.TalonFXIDs.PIVOT, generateMotorConfig(), generateSysidConfig());

		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(pivot.getMotor().getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6LatencySignal positionSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(
			pivot.getMotor().getPosition(),
			velocitySignal,
			GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS
		);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(pivot.getMotor().getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(pivot.getMotor().getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		return new PivotStuff(logPath, pivot, positionRequest, positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

	protected static PivotStuff generateSimulationPivotStuff(String logPath) {
		Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0).withEnableFOC(true));

		SingleJointedArmSim pivotSim = new SingleJointedArmSim(
				DCMotor.getFalcon500Foc(1),
				PivotConstants.GEAR_RATIO,
				SingleJointedArmSim.estimateMOI(
						0.418,
						3
				),
				0.418,
				PivotConstants.BACKWARD_ANGLE_LIMIT.getRadians(),
				PivotConstants.FORWARD_ANGLE_LIMIT.getRadians(),
				false,
				Rotation2d.fromDegrees(16).getRadians()
		);

		SingleJointedArmSimulation pivotSimulation = new SingleJointedArmSimulation(pivotSim, PivotConstants.GEAR_RATIO);

		TalonFXMotor pivot = new TalonFXMotor(
				logPath,
				IDs.TalonFXIDs.PIVOT,
				generateMotorConfig(),
				generateSysidConfig(),
				pivotSimulation
		);

		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
				.generatePhoenix6Signal(pivot.getMotor().getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6LatencySignal positionSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(
				pivot.getMotor().getPosition(),
				velocitySignal,
				GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
				AngleUnit.ROTATIONS
		);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
				.generatePhoenix6Signal(pivot.getMotor().getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
				.generatePhoenix6Signal(pivot.getMotor().getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		return new PivotStuff(logPath, pivot, positionRequest, positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

}
