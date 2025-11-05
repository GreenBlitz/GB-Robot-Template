package frc.robot.subsystems.arm;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class ArmBuilder {

	public DynamicMotionMagicArm create(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		int armId,
		Voltage stepUpVoltage,
		Time timeout,
		Velocity<VoltageUnit> rampUp,
		double JKgMetersSquared,
		double gearing,
		Rotation2d maxAcceleration,
		Rotation2d maxVelocity,
		double feedForward,
		double rotorRatio,
		double mechanismRatio,
		double kP,
		double kI,
		double kD,
		double kG,
		double kV,
		double kA
	) {
		TalonFXMotor arm = new TalonFXMotor(
			logPath + "/Arm",
			new Phoenix6DeviceID(armId),
			new SysIdRoutine.Config(),
			new SimpleMotorSimulation(
				new DCMotorSim(
					LinearSystemId
						.createDCMotorSystem(DCMotor.getKrakenX60Foc(talonFXFollowerConfig.followerIDs.length + 1), JKgMetersSquared, gearing),
					DCMotor.getKrakenX60Foc(talonFXFollowerConfig.followerIDs.length + 1)
				)
			)
		);


		Phoenix6AngleSignal velocity = Phoenix6SignalBuilder
			.build(arm.getDevice().getVelocity(), ArmConstants.defaultFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);

		Phoenix6LatencySignal position = Phoenix6SignalBuilder
			.build(arm.getDevice().getPosition(), velocity, ArmConstants.defaultFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);

		Phoenix6DoubleSignal voltage = Phoenix6SignalBuilder
			.build(arm.getDevice().getMotorVoltage(), ArmConstants.defaultFrequency, BusChain.ROBORIO);

		Phoenix6DoubleSignal current = Phoenix6SignalBuilder
			.build(arm.getDevice().getStatorCurrent(), ArmConstants.defaultFrequency, BusChain.ROBORIO);

		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);

		IDynamicMotionMagicRequest positionRequest = Phoenix6RequestBuilder
			.build(new DynamicMotionMagicVoltage(position.getLatestValue().getRotations(), 0, 0, 0), 0, true);
		positionRequest.withMaxAccelerationRotation2dPerSecondSquared(maxAcceleration);
		positionRequest.withMaxVelocityRotation2dPerSecond(maxVelocity);
		positionRequest.withArbitraryFeedForward(feedForward);
		TalonFXConfiguration configuration = configuration(rotorRatio, mechanismRatio, kP, kG, kI, kD);
		arm.applyConfiguration(configuration);
		SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(
			new SysIdRoutine.Config(rampUp, stepUpVoltage, timeout, state -> SignalLogger.writeString("state", state.toString())),
			true
		);
		return new DynamicMotionMagicArm(
			logPath,
			arm,
			velocity,
			position,
			voltage,
			current,
			voltageRequest,
			positionRequest,
			maxAcceleration,
			maxVelocity,
			sysIdConfigInfo
		);
	}


	public Arm create(
		String logPath,
		Velocity<VoltageUnit> rampUp,
		Voltage stepUpVoltage,
		Time timeout,
		TalonFXFollowerConfig talonFXFollowerConfig,
		int armId,
		double gearing,
		double JKgMetersSquared,
		double feedforward,
		double rotorRatio,
		double mechanismRatio,
		double kP,
		double kI,
		double kD,
		double kG
	) {
		TalonFXMotor arm = new TalonFXMotor(
			logPath + "/Arm",
			new Phoenix6DeviceID(armId),
			new SysIdRoutine.Config(),
			new SimpleMotorSimulation(
				new DCMotorSim(
					LinearSystemId
						.createDCMotorSystem(DCMotor.getKrakenX60Foc(talonFXFollowerConfig.followerIDs.length + 1), JKgMetersSquared, gearing),
					DCMotor.getKrakenX60Foc(talonFXFollowerConfig.followerIDs.length + 1)
				)
			)
		);

		Phoenix6AngleSignal velocity = Phoenix6SignalBuilder
			.build(arm.getDevice().getVelocity(), ArmConstants.defaultFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);

		Phoenix6LatencySignal position = Phoenix6SignalBuilder
			.build(arm.getDevice().getPosition(), velocity, ArmConstants.defaultFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);

		Phoenix6DoubleSignal voltage = Phoenix6SignalBuilder
			.build(arm.getDevice().getMotorVoltage(), ArmConstants.defaultFrequency, BusChain.ROBORIO);

		Phoenix6DoubleSignal current = Phoenix6SignalBuilder
			.build(arm.getDevice().getStatorCurrent(), ArmConstants.defaultFrequency, BusChain.ROBORIO);

		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);

		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder
			.build(new MotionMagicVoltage(position.getLatestValue().getRotations()), feedforward, true);
		arm.applyConfiguration(configuration(rotorRatio, mechanismRatio, kP, kG, kI, kD));
		SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(
			new SysIdRoutine.Config(rampUp, stepUpVoltage, timeout, state -> SignalLogger.writeString("state", state.toString())),
			true
		);

		return new Arm(logPath, arm, velocity, position, voltage, current, voltageRequest, positionRequest, sysIdConfigInfo);
	}

	private static TalonFXConfiguration configuration(double rotorRatio, double mechanismRatio, double kP, double kG, double kI, double kD) {
		TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
		talonFXConfiguration.Feedback.RotorToSensorRatio = rotorRatio;
		talonFXConfiguration.Feedback.SensorToMechanismRatio = mechanismRatio;
		talonFXConfiguration.Slot0.kP = kP;
		talonFXConfiguration.Slot0.kI = kI;
		talonFXConfiguration.Slot0.kD = kD;
		talonFXConfiguration.Slot0.kG = kG;
		talonFXConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
		talonFXConfiguration.CurrentLimits.withStatorCurrentLimitEnable(true);
		talonFXConfiguration.CurrentLimits.withStatorCurrentLimit(ArmConstants.CURRENT_LIMIT);
		talonFXConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
		return talonFXConfiguration;
	}


}
