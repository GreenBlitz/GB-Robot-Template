package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class ArmBuilder {

	public static DynamicMotionMagicArm createDynamicMotionMagicArm(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		Phoenix6DeviceID deviceID,
		SysIdCalibrator.SysIdConfigInfo sysIdCalibratorConfigInfo,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
        Rotation2d forwardSoftwareLimits,
        Rotation2d backwardSoftwareLimits,
		double currentLimit,
		int signalFrequency,
		double JkGMeterSquared,
        double armLengthMeters,
		boolean inverted,
        double arbitraryFeedForward,
        Rotation2d defaultMaxAccelerationRotation2dPerSecondSquare,
        Rotation2d defaultMaxVelocityRotation2dPerSecond
	) {
		TalonFXMotor motor = new TalonFXMotor(
                logPath,
                deviceID,
                talonFXFollowerConfig,
                sysIdCalibratorConfigInfo.config(),
                simulationGenerator(talonFXFollowerConfig,JkGMeterSquared,feedbackConfigs.RotorToSensorRatio*feedbackConfigs.SensorToMechanismRatio,armLengthMeters,backwardSoftwareLimits.getRadians(),forwardSoftwareLimits.getRadians())
        );

        ArmSignals signals = getSignals(motor, signalFrequency, deviceID.busChain());

		Phoenix6Request<Double> voltageRequest = voltageRequest();

		IDynamicMotionMagicRequest positionRequest = Phoenix6RequestBuilder.build(
			new DynamicMotionMagicVoltage(
				signals.positionSignal().getLatestValue().getRotations(),
				defaultMaxVelocityRotation2dPerSecond.getRotations(),
				defaultMaxAccelerationRotation2dPerSecondSquare.getRotations(),
				0
			),
			arbitraryFeedForward,
			true
		);
		TalonFXConfiguration configuration = generateConfiguration(
			feedbackConfigs,
			simulationSlotsConfig,
			realSlotsConfig,
			inverted,
            forwardSoftwareLimits,
            backwardSoftwareLimits,
			currentLimit
		);
        addMotionMagicConfig(configuration,defaultMaxVelocityRotation2dPerSecond,defaultMaxAccelerationRotation2dPerSecondSquare);
		motor.applyConfiguration(configuration);

		return new DynamicMotionMagicArm(
			logPath,
			motor,
			signals,
			voltageRequest,
			positionRequest,
			defaultMaxAccelerationRotation2dPerSecondSquare,
			defaultMaxVelocityRotation2dPerSecond,
			sysIdCalibratorConfigInfo,
			configuration.Slot0.kG
		);
	}

	public static Arm createMotionMagicArm(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		Phoenix6DeviceID deviceID,
		SysIdCalibrator.SysIdConfigInfo sysIdCalibratorConfigInfo,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
        Rotation2d forwardSoftwareLimits,
        Rotation2d backwardSoftwareLimits,
		double currentLimit,
		int signalFrequency,
		double JkGMeterSquared,
        double armLengthMeters,
		boolean inverted,
        double arbitraryFeedForward,
        Rotation2d defaultMaxAccelerationRotation2dPerSecondSquare,
        Rotation2d defaultMaxVelocityRotation2dPerSecond
        ) {
		TalonFXMotor motor = new TalonFXMotor(
			logPath,
			deviceID,
			talonFXFollowerConfig,
			sysIdCalibratorConfigInfo.config(),
			simulationGenerator(talonFXFollowerConfig,JkGMeterSquared,feedbackConfigs.RotorToSensorRatio*feedbackConfigs.SensorToMechanismRatio,armLengthMeters,backwardSoftwareLimits.getRadians(),forwardSoftwareLimits.getRadians())
		);

		ArmSignals signals = getSignals(motor, signalFrequency, deviceID.busChain());

		Phoenix6Request<Double> voltageRequest = voltageRequest();

		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder
			.build(new MotionMagicVoltage(signals.positionSignal().getLatestValue().getRotations()), arbitraryFeedForward, true);
		TalonFXConfiguration configuration = (generateConfiguration(
			feedbackConfigs,
			simulationSlotsConfig,
			realSlotsConfig,
			inverted,
            forwardSoftwareLimits,
            backwardSoftwareLimits,
			currentLimit
		));
        addMotionMagicConfig(configuration,defaultMaxVelocityRotation2dPerSecond,defaultMaxAccelerationRotation2dPerSecondSquare);

        motor.applyConfiguration(configuration);

		return new Arm(logPath, motor, signals, voltageRequest, positionRequest, sysIdCalibratorConfigInfo, configuration.Slot0.kG);
	}

    public static Arm createArm(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		Phoenix6DeviceID deviceID,
		SysIdCalibrator.SysIdConfigInfo sysIdCalibratorConfigInfo,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
        Rotation2d forwardSoftwareLimits,
        Rotation2d backwardSoftwareLimits,
		double currentLimit,
		int signalFrequency,
		double JkGMeterSquared,
        double armLengthMeters,
		boolean inverted,
        double arbitraryFeedForward
    ) {
		TalonFXMotor motor = new TalonFXMotor(
			logPath,
			deviceID,
			talonFXFollowerConfig,
			sysIdCalibratorConfigInfo.config(),
			simulationGenerator(talonFXFollowerConfig,JkGMeterSquared,feedbackConfigs.RotorToSensorRatio*feedbackConfigs.SensorToMechanismRatio,armLengthMeters,backwardSoftwareLimits.getRadians(),forwardSoftwareLimits.getRadians())
		);

		ArmSignals signals = getSignals(motor, signalFrequency, deviceID.busChain());

		Phoenix6Request<Double> voltageRequest = voltageRequest();

        Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder
                .build(new PositionVoltage(signals.positionSignal().getLatestValue().getRotations()), arbitraryFeedForward, true);

        TalonFXConfiguration configuration = (generateConfiguration(
			feedbackConfigs,
			simulationSlotsConfig,
			realSlotsConfig,
			inverted,
            forwardSoftwareLimits,
            backwardSoftwareLimits,
			currentLimit
		));
		motor.applyConfiguration(configuration);

		return new Arm(logPath, motor, signals, voltageRequest, positionRequest, sysIdCalibratorConfigInfo, configuration.Slot0.kG);
	}

	private static TalonFXConfiguration generateConfiguration(
		FeedbackConfigs feedbackConfigs,
		Slot0Configs simulationConfigSlots,
		Slot0Configs realConfigSlots,
		boolean invertedValue,
        Rotation2d forwardLimitSwitch,
        Rotation2d backwardLimitSwitch,
		double currentLimit
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

        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonFXConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimitSwitch.getRotations();
        talonFXConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = backwardLimitSwitch.getRotations();

        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit =currentLimit;

        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        if (invertedValue)
		    talonFXConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		return talonFXConfiguration;
	}

    private static void addMotionMagicConfig(TalonFXConfiguration config,Rotation2d maxVelocity,Rotation2d maxAcceleration){
        config.MotionMagic.MotionMagicAcceleration = maxAcceleration.getRotations();
        config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity.getRotations();
    }

	private static SingleJointedArmSimulation simulationGenerator(

		TalonFXFollowerConfig followerConfig,
		double JkGMeterSquared,
		double gearing,
		double armLengthMeters,
		double minAngleRads,
		double maxAngleRads
	) {
		return new SingleJointedArmSimulation(
			new SingleJointedArmSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length + 1), JkGMeterSquared, gearing),
				DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length + 1),
				gearing,
				armLengthMeters,
				minAngleRads,
				maxAngleRads,
				false,
				0
			),
			gearing
		);
	}

	private static Phoenix6Request<Double> voltageRequest() {
		return Phoenix6RequestBuilder.build(new VoltageOut(0), true);
	}

    private static ArmSignals getSignals(TalonFXMotor motor,int signalFrequency,BusChain busChain){
        Phoenix6AngleSignal velocity = Phoenix6SignalBuilder.build(motor.getDevice().getVelocity(), signalFrequency, AngleUnit.ROTATIONS, busChain);
        return new ArmSignals(
                Phoenix6SignalBuilder.build(motor.getDevice().getMotorVoltage(), signalFrequency, busChain),
                Phoenix6SignalBuilder.build(motor.getDevice().getStatorCurrent(), signalFrequency, busChain),
                velocity,
                Phoenix6SignalBuilder
                        .build(motor.getDevice().getPosition(), velocity, signalFrequency,AngleUnit.ROTATIONS,busChain)
        );
    }

}
