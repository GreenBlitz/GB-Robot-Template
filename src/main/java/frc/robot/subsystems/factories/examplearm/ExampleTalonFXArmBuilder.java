package frc.robot.subsystems.factories.examplearm;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.examplearm.ExampleArm;
import frc.utils.AngleUnit;

import static frc.robot.IDs.TalonFXIDs.ARM_DEVICE_ID;

public class ExampleTalonFXArmBuilder {

	static ExampleArm build(String logPath) {
		Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0).withSlot(0).withEnableFOC(true));
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true));

		SysIdRoutine.Config sysIdConfig = buildSysidConfig();
		TalonFXMotor motor = new TalonFXMotor(logPath, ARM_DEVICE_ID, sysIdConfig);
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
		config.Slot0.kP = 0;
		config.Slot0.kI = 0;
		config.Slot0.kD = 0;
		config.Slot0.kS = 0;
		config.Slot0.kG = 0;

		config.CurrentLimits.withSupplyCurrentLimit(30);
		config.Feedback.withSensorToMechanismRatio(1);
		config.MotorOutput.withPeakForwardDutyCycle(0.9);
		config.MotorOutput.withPeakReverseDutyCycle(-0.9);
		config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(90);
		config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-90);

		return config;
	}

}
