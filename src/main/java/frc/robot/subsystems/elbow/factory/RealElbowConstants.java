package frc.robot.subsystems.elbow.factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motor.TalonFXMotor;
import frc.robot.hardware.phoenix6.motor.TalonFXWrapper;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.rev.request.SparkMaxRequest;
import frc.robot.hardware.rev.request.SparkMaxRequestBuilder;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.elbow.ElbowStuff;
import frc.utils.AngleUnit;

import java.util.function.Function;

public class RealElbowConstants {

	private static final double KS = 0.15;
	private static final double KG = 0.2;
	private static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(KS, KG, 0);

	private static final int POSITION_PID_SLOT = 0;

	private static SparkMaxRequest<Rotation2d> generatePositionRequest() {
		Function<Rotation2d, Double> feedforwardCalculation = setPoint -> FEEDFORWARD.calculate(setPoint.getRadians(), 0);
		return SparkMaxRequestBuilder.build(new Rotation2d(), CANSparkBase.ControlType.kPosition, POSITION_PID_SLOT, feedforwardCalculation);
	}

	private static SparkMaxRequest<Double> generateVoltageRequest() {
		return SparkMaxRequestBuilder.build(0.0, CANSparkBase.ControlType.kVoltage, 0);
	}

	private static void configMotor(CANSparkMax motor) {
		motor.setInverted(true);
		motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) ElbowConstants.FORWARD_LIMIT.getRotations());
		motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
		motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) ElbowConstants.BACKWARD_LIMIT.getRotations());
		motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
		motor.setSmartCurrentLimit(40);
		motor.getPIDController().setP(5.5, POSITION_PID_SLOT);
		motor.getPIDController().setD(0.5, POSITION_PID_SLOT);
		motor.getEncoder().setPositionConversionFactor(ElbowConstants.GEAR_RATIO);
		motor.getEncoder().setPositionConversionFactor(ElbowConstants.GEAR_RATIO);
	}

	protected static ElbowStuff generateElbowStuff(String logPath) {
		SparkMaxWrapper motor = new SparkMaxWrapper(IDs.CANSparkMAXs.ELBOW);
		configMotor(motor);

		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", () -> motor.getEncoder().getPosition(), AngleUnit.ROTATIONS);
		SuppliedAngleSignal velocitySignal = new SuppliedAngleSignal("velocity", () -> motor.getEncoder().getVelocity(), AngleUnit.ROTATIONS);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("output current", motor::getOutputCurrent);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", motor::getVoltage);

		BrushlessSparkMAXMotor elbow = new BrushlessSparkMAXMotor(logPath, motor, new SysIdRoutine.Config());
		return new ElbowStuff(
			logPath,
			elbow,
			generatePositionRequest(),
			generateVoltageRequest(),
			positionSignal,
			velocitySignal,
			currentSignal,
			voltageSignal
		);
	}

	private static TalonFXConfiguration generateConfiguration(){
		TalonFXConfiguration configuration = new TalonFXConfiguration();
		return configuration;
	}

	private static SysIdRoutine.Config generateConfig(){
		SysIdRoutine.Config config = new SysIdRoutine.Config();
		return config;
	}

	protected static ElbowStuff generateSimulationElbowStuff(String logPath){
		TalonFXWrapper wrapper = new TalonFXWrapper(0);

		Phoenix6AngleSignal positionSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(wrapper.getPosition(), 60, AngleUnit.DEGREES);
		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder.generatePhoenix6Signal(wrapper.getVelocity(), 60, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(wrapper.getSupplyCurrent(), 60);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(wrapper.getMotorVoltage(), 60);

		SingleJointedArmSim armSim = new SingleJointedArmSim(
				DCMotor.getFalcon500(1),
				ElbowConstants.GEAR_RATIO,
				SingleJointedArmSim.estimateMOI(
						0.44,
						0.44
				),
				0.44,
				Rotation2d.fromDegrees(-81).getRadians(),
				Rotation2d.fromDegrees(90).getRadians(),
				false,
				Rotation2d.fromDegrees(0).getRadians()
		);

		SingleJointedArmSimulation simulation = new SingleJointedArmSimulation(
				armSim,
				ElbowConstants.GEAR_RATIO
		);

		TalonFXMotor motor = new TalonFXMotor(
				logPath,
				new Phoenix6DeviceID(0),
				generateConfiguration(),
				generateConfig(),
				simulation
		);
		return new ElbowStuff(
				logPath,
				motor,

		)
	}

}
