package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class ArmBuilder {

	public static DynamicMotionMagicArm create(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		Phoenix6DeviceID deviceID,
		SysIdCalibrator.SysIdConfigInfo sysIdCalibratorConfigInfo,
		Rotation2d maxAcceleration,
		Rotation2d maxVelocity,
		double arbitraryFeedForward,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
		int currentLimit,
		BusChain busChain,
		int signalFrequency,
        double JkGMeterSquared,
		InvertedValue inverted
	) {
		TalonFXMotor motor = motorGenerator(deviceID, logPath, talonFXFollowerConfig, JkGMeterSquared,feedbackConfigs.RotorToSensorRatio*feedbackConfigs.SensorToMechanismRatio, sysIdCalibratorConfigInfo);

		ArmSignals signals = new ArmSignals(motor, signalFrequency, busChain);

		Phoenix6Request<Double> voltageRequest = voltageRequest();

		IDynamicMotionMagicRequest positionRequest = Phoenix6RequestBuilder.build(
			new DynamicMotionMagicVoltage(
				signals.positionSignal().getLatestValue().getRotations(),
				maxVelocity.getRotations(),
				maxAcceleration.getRotations(),
				0
			),
			arbitraryFeedForward,
			true
		);
		positionRequest.withMaxAccelerationRotation2dPerSecondSquared(maxAcceleration);
		positionRequest.withMaxVelocityRotation2dPerSecond(maxVelocity);
		positionRequest.withArbitraryFeedForward(arbitraryFeedForward);
		TalonFXConfiguration configuration = generateConfiguration(
			feedbackConfigs,
			simulationSlotsConfig,
			realSlotsConfig,
			inverted,
			currentLimit
		);
		motor.applyConfiguration(configuration);

		return new DynamicMotionMagicArm(
			logPath,
			motor,
			signals,
			voltageRequest,
			positionRequest,
			maxAcceleration,
			maxVelocity,
			sysIdCalibratorConfigInfo,
			configuration.Slot0.kG
		);
	}

	public static Arm create(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		Phoenix6DeviceID deviceID,
		SysIdCalibrator.SysIdConfigInfo sysIdCalibratorConfigInfo,
		double arbitraryFeedForward,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
		int currentLimit,
		BusChain busChain,
		int signalFrequency,
        double JkGMeterSquared,
		InvertedValue inverted
	) {
		TalonFXMotor motor = motorGenerator(deviceID, logPath, talonFXFollowerConfig, JkGMeterSquared,feedbackConfigs.RotorToSensorRatio*feedbackConfigs.SensorToMechanismRatio,sysIdCalibratorConfigInfo);

		ArmSignals signals = new ArmSignals(motor, signalFrequency, busChain);

		Phoenix6Request<Double> voltageRequest = voltageRequest();

		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder
			.build(new MotionMagicVoltage(signals.positionSignal().getLatestValue().getRotations()), arbitraryFeedForward, true);
		TalonFXConfiguration configuration = (generateConfiguration(
			feedbackConfigs,
			simulationSlotsConfig,
			realSlotsConfig,
			inverted,
			currentLimit
		));
		motor.applyConfiguration(configuration);

		return new Arm(
			logPath,
			motor,
			signals,
			voltageRequest,
			positionRequest,
			sysIdCalibratorConfigInfo,
			configuration.Slot0.kG
		);
	}

	private static TalonFXConfiguration generateConfiguration(
		FeedbackConfigs feedbackConfigs,
		Slot0Configs simulationConfigSlots,
		Slot0Configs realConfigSlots,
		InvertedValue invertedValue,
		int currentLimit
	) {
		TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
		switch (Robot.ROBOT_TYPE) {
			case REAL -> {
				talonFXConfiguration.Slot0 = realConfigSlots;
			}
			case SIMULATION -> {
				talonFXConfiguration.Slot0 = simulationConfigSlots;
			}
		}
		talonFXConfiguration.Feedback = feedbackConfigs;


		talonFXConfiguration.MotorOutput.withInverted(invertedValue);
		talonFXConfiguration.CurrentLimits.withStatorCurrentLimitEnable(true);
		talonFXConfiguration.CurrentLimits.withStatorCurrentLimit(currentLimit);
		talonFXConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
		return talonFXConfiguration;
	}

	private static TalonFXMotor motorGenerator(
		Phoenix6DeviceID deviceID,
		String logPath,
		TalonFXFollowerConfig followerConfig,
        double JkGMeterSquared,
        double gearing,
		SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo
	) {
        return new TalonFXMotor(logPath, deviceID, followerConfig, sysIdConfigInfo.config(),new SimpleMotorSimulation(new DCMotorSim(LinearSystemId
                .createDCMotorSystem(DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length + 1), JkGMeterSquared, gearing),
                DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length + 1)
        )));
	}

	private static Phoenix6Request<Double> voltageRequest() {
		return Phoenix6RequestBuilder.build(new VoltageOut(0), true);
	}


}
