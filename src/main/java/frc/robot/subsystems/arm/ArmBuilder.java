package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
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
		double feedForward,
		FeedbackConfigs feedbackConfigs,
		TalonFXConfiguration realSlotsConfig,
		TalonFXConfiguration simulationSlotsConfig,
		double calibrationMaxPower,
        Rotation2d defaultPositionTolerance,
		int currentLimit,
		int signalFrequency,
		BusChain busChain
	) {
		TalonFXMotor motor = motorGenerator(deviceID, logPath, talonFXFollowerConfig, sysIdCalibratorConfigInfo);

		ArmSignals signals = new ArmSignals(motor, signalFrequency, busChain);

		Phoenix6Request<Double> voltageRequest = voltageRequest();

		IDynamicMotionMagicRequest positionRequest = Phoenix6RequestBuilder
			.build(new DynamicMotionMagicVoltage(signals.positionSignal().getLatestValue().getRotations(), maxVelocity.getRotations(), maxAcceleration.getRotations(), 0), 0, true);
		positionRequest.withMaxAccelerationRotation2dPerSecondSquared(maxAcceleration);
		positionRequest.withMaxVelocityRotation2dPerSecond(maxVelocity);
		positionRequest.withArbitraryFeedForward(feedForward);
		TalonFXConfiguration configuration = generateConfiguration(feedbackConfigs, simulationSlotsConfig, realSlotsConfig, currentLimit);
		motor.applyConfiguration(configuration);

		return new DynamicMotionMagicArm(
			logPath,
			motor,
			signals.velocitySignal(),
			signals.positionSignal(),
			signals.voltageSignal(),
			signals.currentSignal(),
			voltageRequest,
			positionRequest,
			maxAcceleration,
			maxVelocity,
			sysIdCalibratorConfigInfo,
			configuration.Slot0.kG,
			calibrationMaxPower,
            defaultPositionTolerance
		);
	}

	public static Arm create(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		Phoenix6DeviceID deviceID,
		SysIdCalibrator.SysIdConfigInfo sysIdCalibratorConfigInfo,
		double feedforward,
		FeedbackConfigs feedbackConfigs,
		TalonFXConfiguration realSlotsConfig,
		TalonFXConfiguration simulationSlotsConfig,
		double calibrationMaxPower,
		int currentLimit,
		int signalFrequency,
        Rotation2d defaultPositionTolerance,
		BusChain busChain
	) {
		TalonFXMotor motor = motorGenerator(deviceID, logPath, talonFXFollowerConfig, sysIdCalibratorConfigInfo);

		ArmSignals signals = new ArmSignals(motor, signalFrequency, busChain);

		Phoenix6Request<Double> voltageRequest = voltageRequest();

		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder
			.build(new MotionMagicVoltage(signals.positionSignal().getLatestValue().getRotations()), feedforward, true);
		TalonFXConfiguration configuration = (generateConfiguration(feedbackConfigs, simulationSlotsConfig, realSlotsConfig, currentLimit));
		motor.applyConfiguration(configuration);

		return new Arm(
			logPath,
			motor,
			signals.velocitySignal(),
			signals.positionSignal(),
			signals.voltageSignal(),
			signals.currentSignal(),
			voltageRequest,
			positionRequest,
			sysIdCalibratorConfigInfo,
			configuration.Slot0.kG,
			calibrationMaxPower,
            defaultPositionTolerance
		);
	}

	private static TalonFXConfiguration generateConfiguration(
		FeedbackConfigs feedbackConfigs,
		TalonFXConfiguration simulationConfigSlots,
		TalonFXConfiguration realConfigSlots,
		int currentLimit
	) {
		TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
		switch (Robot.ROBOT_TYPE) {
			case REAL -> {
				talonFXConfiguration = realConfigSlots;
			}
			case SIMULATION -> {
				talonFXConfiguration = simulationConfigSlots;
			}
		}
		talonFXConfiguration.Feedback = feedbackConfigs;


		talonFXConfiguration.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
		talonFXConfiguration.CurrentLimits.withStatorCurrentLimitEnable(true);
		talonFXConfiguration.CurrentLimits.withStatorCurrentLimit(currentLimit);
		talonFXConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
		return talonFXConfiguration;
	}

	private static TalonFXMotor motorGenerator(
		Phoenix6DeviceID deviceID,
		String logPath,
		TalonFXFollowerConfig followerConfig,
		SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo
	) {
		return new TalonFXMotor(logPath, deviceID, followerConfig, sysIdConfigInfo.config());
	}

	private static Phoenix6Request<Double> voltageRequest() {
		return Phoenix6RequestBuilder.build(new VoltageOut(0), true);
	}


}
