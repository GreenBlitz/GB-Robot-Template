package frc.robot.subsystems.elbow.factory;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motor.TalonFXMotor;
import frc.robot.hardware.phoenix6.motor.TalonFXWrapper;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.elbow.ElbowStuff;
import frc.utils.AngleUnit;

public class SimulationElbowConstants {

	public static final double SIMULATION_GEAR_RATIO = 1 / ElbowConstants.GEAR_RATIO;


	private static TalonFXConfiguration generateConfiguration() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();
		configuration.Slot0.withKP(2);
		SoftwareLimitSwitchConfigs limitSwitchConfigs = new SoftwareLimitSwitchConfigs();
		limitSwitchConfigs.withReverseSoftLimitThreshold(ElbowConstants.BACKWARD_LIMIT.getRotations());
		limitSwitchConfigs.withForwardSoftLimitThreshold(ElbowConstants.FORWARD_LIMIT.getRotations());
		limitSwitchConfigs.ForwardSoftLimitEnable = true;
		limitSwitchConfigs.ReverseSoftLimitEnable = true;
		configuration.withSoftwareLimitSwitch(limitSwitchConfigs);
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;
		configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
		configuration.CurrentLimits.StatorCurrentLimit = 40;
		configuration.CurrentLimits.SupplyCurrentLimit = 40;
		return configuration;
	}

	private static SysIdRoutine.Config generateConfig() {
		return new SysIdRoutine.Config();
	}

	protected static ElbowStuff generateSimulationElbowStuff(String logPath) {
		Phoenix6DeviceID deviceID = new Phoenix6DeviceID(IDs.CANSparkMAXs.ELBOW.id());
		TalonFXWrapper wrapper = new TalonFXWrapper(deviceID);

		Phoenix6AngleSignal positionSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(wrapper.getPosition(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.DEGREES);
		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(wrapper.getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(wrapper.getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(wrapper.getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		SingleJointedArmSim armSim = new SingleJointedArmSim(
			DCMotor.getNEO(1),
			SIMULATION_GEAR_RATIO,
			SingleJointedArmSim.estimateMOI(0.44, 0.44),
			0.44,
			ElbowConstants.BACKWARD_LIMIT.getRadians(),
			ElbowConstants.FORWARD_LIMIT.getRadians(),
			false,
			Rotation2d.fromDegrees(0).getRadians()
		);

		SingleJointedArmSimulation simulation = new SingleJointedArmSimulation(armSim, SIMULATION_GEAR_RATIO);

		Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0));
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0));

		TalonFXMotor motor = new TalonFXMotor(logPath, deviceID, generateConfiguration(), generateConfig(), simulation);

		return new ElbowStuff(logPath, motor, positionRequest, voltageRequest, positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

}
