package frc.robot.subsystems.factories.examplearm;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.examplearm.ExampleArm;
import frc.utils.AngleUnit;

public class ExampleTalonFXArmBuilder {

	public static final double kGEAR_RATIO = 1;

	static ExampleArm build(String logPath) {
		Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0).withSlot(0).withEnableFOC(true));
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true));

		SingleJointedArmSim armSim = new SingleJointedArmSim(
			LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, kGEAR_RATIO),
			DCMotor.getKrakenX60(1),
			1,
			1.5,
			Rotation2d.fromDegrees(-90).getRadians(),
			Rotation2d.fromDegrees(90).getRadians(),
			false,
			Rotation2d.fromDegrees(0).getRadians()
		);
		SingleJointedArmSimulation armSimulation = new SingleJointedArmSimulation(armSim, 1);

		SysIdRoutine.Config sysIdConfig = buildSysidConfig();
		TalonFXMotor motor = new TalonFXMotor(logPath, IDs.TalonFXIDs.EXAMPLE_ARM_DEVICE_ID, sysIdConfig, armSimulation);
		motor.applyConfiguration(buildTalonFXConfiguration());

		Phoenix6AngleSignal positionSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		return new ExampleArm(logPath, motor, positionRequest, voltageRequest, positionSignal, voltageSignal);
	}

	private static SysIdRoutine.Config buildSysidConfig() {
		return new SysIdRoutine.Config(null, null, null, state -> SignalLogger.writeString("state", state.toString()));
	}

	private static TalonFXConfiguration buildTalonFXConfiguration() {
		TalonFXConfiguration config = new TalonFXConfiguration();
		config.Slot0.kP = 1;
		config.Slot0.kI = 0;
		config.Slot0.kD = 0;
		config.Slot0.kS = 0;
		config.Slot0.kG = 0;

		config.CurrentLimits.withSupplyCurrentLimit(30);
		config.MotorOutput.withPeakForwardDutyCycle(0.9);
		config.MotorOutput.withPeakReverseDutyCycle(-0.9);
		config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(90);
		config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-90);
		config.Feedback.withSensorToMechanismRatio(kGEAR_RATIO);
		config.Feedback.withRotorToSensorRatio(1);

		return config;
	}

}
