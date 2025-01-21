package frc.robot.subsystems.factories.armfactory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.constants.MathConstants;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.arm.Arm;
import frc.utils.math.AngleUnit;

public class TalonFXArmBuilder {

	public static final double GEAR_RATIO = 15;

	public static Arm build(String logPath) {
		Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0).withSlot(0).withEnableFOC(true));
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true));

		SingleJointedArmSim armSim = new SingleJointedArmSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, GEAR_RATIO),
				DCMotor.getKrakenX60(1),
				GEAR_RATIO,
				1,
				MathConstants.QUARTER_CIRCLE.unaryMinus().getRadians(),
				MathConstants.QUARTER_CIRCLE.getRadians(),
				false,
				0
		);
		SingleJointedArmSimulation armSimulation = new SingleJointedArmSimulation(armSim, GEAR_RATIO);

		TalonFXMotor motor = new TalonFXMotor(logPath, IDs.TalonFXIDs.ARM_DEVICE_ID, buildSysidConfig(), armSimulation);
		motor.applyConfiguration(buildTalonFXConfiguration());

		Phoenix6AngleSignal positionSignal = Phoenix6SignalBuilder
				.generatePhoenix6Signal(motor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
				.generatePhoenix6Signal(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		return new Arm(logPath, motor, positionRequest, voltageRequest, positionSignal, voltageSignal);
	}


	public static SysIdRoutine.Config buildSysidConfig() {
		return new SysIdRoutine.Config(null, null, null, state -> SignalLogger.writeString("state", state.toString()));
	}

	private static TalonFXConfiguration buildTalonFXConfiguration() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Slot0.kP = 1;
		config.Slot0.kI = 0.8;
		config.Slot0.kD = 0.8;
		config.Slot0.kS = 0;
		config.Slot0.kG = 0;
		config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

		config.CurrentLimits.SupplyCurrentLimit = 30;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = 40;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MathConstants.QUARTER_CIRCLE.getRotations();
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MathConstants.QUARTER_CIRCLE.unaryMinus().getRotations();
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

		config.Feedback.RotorToSensorRatio = GEAR_RATIO;
		config.Feedback.SensorToMechanismRatio = 1;

		return config;
	}

}
