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
import frc.utils.calibration.sysid.SysIdCalibrator;

public class ArmBuilder {

	public static DynamicMotionMagicArm createDynamicMotionMagic(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		Phoenix6DeviceID deviceID,
		SysIdCalibrator.SysIdConfigInfo sysIdCalibratorConfigInfo,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
		double currentLimit,
		int signalFrequency,
		double JkGMeterSquared,
        double armLengthMeter,
        double minAngleRads,
        double maxAngleRads,
		InvertedValue inverted,
        double arbitraryFeedForward,
        Rotation2d defaultMaxAccelerationRotation2dPerSecondSquare,
        Rotation2d defaultMaxVelocityRotation2dPerSecond
	) {
		TalonFXMotor motor = new TalonFXMotor(
                logPath,
                deviceID,
                talonFXFollowerConfig,
                sysIdCalibratorConfigInfo.config(),
                simulationGenerator(talonFXFollowerConfig,JkGMeterSquared,feedbackConfigs.RotorToSensorRatio*feedbackConfigs.SensorToMechanismRatio,armLengthMeter,minAngleRads,maxAngleRads)
        );

        ArmSignals signals = ArmSignals.getSignals(motor, signalFrequency, deviceID.busChain());

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

	public static Arm create(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		Phoenix6DeviceID deviceID,
		SysIdCalibrator.SysIdConfigInfo sysIdCalibratorConfigInfo,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
		double currentLimit,
		int signalFrequency,
		double JkGMeterSquared,
        double armLengthMeter,
        double minAngleRads,
        double maxAngleRads,
		InvertedValue inverted,
        double arbitraryFeedForward,
        Rotation2d defaultMaxAccelerationRotation2dPerSecondSquare,
        Rotation2d defaultMaxVelocityRotation2dPerSecond
        ) {
		TalonFXMotor motor = new TalonFXMotor(
			logPath,
			deviceID,
			talonFXFollowerConfig,
			sysIdCalibratorConfigInfo.config(),
			simulationGenerator(talonFXFollowerConfig,JkGMeterSquared,feedbackConfigs.RotorToSensorRatio*feedbackConfigs.SensorToMechanismRatio,armLengthMeter,minAngleRads,maxAngleRads)
		);

		ArmSignals signals = ArmSignals.getSignals(motor, signalFrequency, deviceID.busChain());

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
        addMotionMagicConfig(configuration,defaultMaxVelocityRotation2dPerSecond,defaultMaxAccelerationRotation2dPerSecondSquare);

        motor.applyConfiguration(configuration);

		return new Arm(logPath, motor, signals, voltageRequest, positionRequest, sysIdCalibratorConfigInfo, configuration.Slot0.kG);
	}

    public static Arm create(
		String logPath,
		TalonFXFollowerConfig talonFXFollowerConfig,
		Phoenix6DeviceID deviceID,
		SysIdCalibrator.SysIdConfigInfo sysIdCalibratorConfigInfo,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
		double currentLimit,
		int signalFrequency,
		double JkGMeterSquared,
        double armLengthMeter,
        double minAngleRads,
        double maxAngleRads,
		InvertedValue inverted,
        double arbitraryFeedForward
    ) {
		TalonFXMotor motor = new TalonFXMotor(
			logPath,
			deviceID,
			talonFXFollowerConfig,
			sysIdCalibratorConfigInfo.config(),
			simulationGenerator(talonFXFollowerConfig,JkGMeterSquared,feedbackConfigs.RotorToSensorRatio*feedbackConfigs.SensorToMechanismRatio,armLengthMeter,minAngleRads,maxAngleRads)
		);

		ArmSignals signals = ArmSignals.getSignals(motor, signalFrequency, deviceID.busChain());

		Phoenix6Request<Double> voltageRequest = voltageRequest();

        Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder
                .build(new PositionVoltage(signals.positionSignal().getLatestValue().getRotations()), arbitraryFeedForward, true);

        TalonFXConfiguration configuration = (generateConfiguration(
			feedbackConfigs,
			simulationSlotsConfig,
			realSlotsConfig,
			inverted,
			currentLimit
		));
		motor.applyConfiguration(configuration);

		return new Arm(logPath, motor, signals, voltageRequest, positionRequest, sysIdCalibratorConfigInfo, configuration.Slot0.kG);
	}

	private static TalonFXConfiguration generateConfiguration(
		FeedbackConfigs feedbackConfigs,
		Slot0Configs simulationConfigSlots,
		Slot0Configs realConfigSlots,
		InvertedValue invertedValue,
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


		talonFXConfiguration.MotorOutput.Inverted = invertedValue;
		talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
		talonFXConfiguration.CurrentLimits.StatorCurrentLimit =currentLimit;
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


}
