package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import com.ctre.phoenix6.controls.VoltageOut;

public class FlyWheelBuilder {

	public static FlyWheel generate(String logPath, Phoenix6DeviceID IDSRightMotor, Phoenix6DeviceID IDLeftMotor) {
		TalonFXMotor rightMotor = new TalonFXMotor(logPath + "/right", IDSRightMotor, new SysIdRoutine.Config());
		TalonFXMotor leftMotor = new TalonFXMotor(logPath + "/left", IDLeftMotor, new SysIdRoutine.Config());

		Phoenix6DoubleSignal voltageSignalRightMotor = Phoenix6SignalBuilder
			.build(rightMotor.getDevice().getMotorVoltage(), Constants.DEFAULT_SIGNALS_FREQUENCY, BusChain.ROBORIO);
		Phoenix6DoubleSignal voltageSignalLeftMotor = Phoenix6SignalBuilder
			.build(leftMotor.getDevice().getMotorVoltage(), Constants.DEFAULT_SIGNALS_FREQUENCY, BusChain.ROBORIO);
		Phoenix6AngleSignal velocitySignalRightMotor = Phoenix6SignalBuilder
			.build(rightMotor.getDevice().getVelocity(), Constants.DEFAULT_SIGNALS_FREQUENCY, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		Phoenix6AngleSignal velocitySignalLeftMotor = Phoenix6SignalBuilder
			.build(leftMotor.getDevice().getVelocity(), Constants.DEFAULT_SIGNALS_FREQUENCY, AngleUnit.ROTATIONS, BusChain.ROBORIO);

		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);
		Phoenix6Request<Rotation2d> velocityRequest = Phoenix6RequestBuilder.build(new VelocityVoltage(0), 0, true);
		rightMotor.applyConfiguration(buildConfigRight());
		leftMotor.applyConfiguration(buildConfigLeft());

		return new FlyWheel(
			logPath,
			velocityRequest,
			voltageRequest,
			velocitySignalRightMotor,
			voltageSignalRightMotor,
			velocitySignalLeftMotor,
			voltageSignalLeftMotor,
			rightMotor,
			leftMotor
		);
	}

	public static TalonFXConfiguration buildConfigRight() {
		TalonFXConfiguration LeftMotorConfiguration = new TalonFXConfiguration();
		LeftMotorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.CURRENT_LIMITS;
		LeftMotorConfiguration.Voltage.withPeakForwardVoltage(Constants.FORWARD_VOLTAGE_LIMIT);
		LeftMotorConfiguration.Voltage.withPeakReverseVoltage(Constants.BACKWARD_VOLTAGE_LIMIT);
		return LeftMotorConfiguration;
	}

	public static TalonFXConfiguration buildConfigLeft() {
		TalonFXConfiguration RightMotorConfiguration = new TalonFXConfiguration();
		RightMotorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.CURRENT_LIMITS;
		RightMotorConfiguration.Voltage.withPeakForwardVoltage(Constants.FORWARD_VOLTAGE_LIMIT);
		return RightMotorConfiguration;
	}

}

