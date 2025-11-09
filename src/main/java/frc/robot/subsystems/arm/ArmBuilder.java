package frc.robot.subsystems.arm;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
import frc.robot.hardware.interfaces.IRequest;
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

import static frc.robot.RobotType.SIMULATION;

public class ArmBuilder {

	public DynamicMotionMagicArm create(
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
        int currentLimit,
        int signalFrequency,
        BusChain busChain
    ) {
        TalonFXMotor arm = arm(deviceID,logPath,talonFXFollowerConfig,sysIdCalibratorConfigInfo);

		Phoenix6AngleSignal velocity = Phoenix6SignalBuilder
			.build(arm.getDevice().getVelocity(), signalFrequency, AngleUnit.ROTATIONS, busChain);

		Phoenix6LatencySignal position = Phoenix6SignalBuilder
			.build(arm.getDevice().getPosition(), velocity, signalFrequency, AngleUnit.ROTATIONS, busChain);

		Phoenix6DoubleSignal voltage = Phoenix6SignalBuilder
			.build(arm.getDevice().getMotorVoltage(), signalFrequency,busChain );

		Phoenix6DoubleSignal current = Phoenix6SignalBuilder
			.build(arm.getDevice().getStatorCurrent(), signalFrequency, busChain);

		Phoenix6Request<Double> voltageRequest = voltageRequest();

		IDynamicMotionMagicRequest positionRequest = Phoenix6RequestBuilder
			.build(new DynamicMotionMagicVoltage(position.getLatestValue().getRotations(), 0, 0, 0), 0, true);
		positionRequest.withMaxAccelerationRotation2dPerSecondSquared(maxAcceleration);
		positionRequest.withMaxVelocityRotation2dPerSecond(maxVelocity);
		positionRequest.withArbitraryFeedForward(feedForward);
		TalonFXConfiguration configuration = configuration(feedbackConfigs,simulationSlotsConfig,realSlotsConfig,currentLimit);
		arm.applyConfiguration(configuration);

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
			sysIdCalibratorConfigInfo,
            configuration.Slot0.kG,
            calibrationMaxPower
		);
	}


	public Arm create(
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
        int signalFrequency
	) {
        TalonFXMotor arm = arm(deviceID,logPath,talonFXFollowerConfig,sysIdCalibratorConfigInfo);

		Phoenix6AngleSignal velocity = Phoenix6SignalBuilder
			.build(arm.getDevice().getVelocity(), signalFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);

		Phoenix6LatencySignal position = Phoenix6SignalBuilder
			.build(arm.getDevice().getPosition(), velocity, signalFrequency, AngleUnit.ROTATIONS, BusChain.ROBORIO);

		Phoenix6DoubleSignal voltage = Phoenix6SignalBuilder
			.build(arm.getDevice().getMotorVoltage(), signalFrequency, BusChain.ROBORIO);

		Phoenix6DoubleSignal current = Phoenix6SignalBuilder
			.build(arm.getDevice().getStatorCurrent(), signalFrequency, BusChain.ROBORIO);

		Phoenix6Request<Double> voltageRequest = voltageRequest();

		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder
			.build(new MotionMagicVoltage(position.getLatestValue().getRotations()), feedforward, true);
        TalonFXConfiguration configuration = (configuration(feedbackConfigs,simulationSlotsConfig,realSlotsConfig,currentLimit));
        arm.applyConfiguration(configuration);

		return new Arm(logPath, arm, velocity, position, voltage, current, voltageRequest, positionRequest, sysIdCalibratorConfigInfo,configuration.Slot0.kG,calibrationMaxPower);
	}

	private static TalonFXConfiguration configuration(FeedbackConfigs feedbackConfigs,TalonFXConfiguration simulationConfigSlots,TalonFXConfiguration realConfigSlots,int currentLimit) {
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

    private static TalonFXMotor arm(Phoenix6DeviceID deviceID, String logPath, TalonFXFollowerConfig followerConfig, SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo){
        return new TalonFXMotor(logPath,deviceID,followerConfig,sysIdConfigInfo.config());
    }

    private static Phoenix6Request<Double> voltageRequest(){
        return Phoenix6RequestBuilder.build(new VoltageOut(0), true);
    }




}
