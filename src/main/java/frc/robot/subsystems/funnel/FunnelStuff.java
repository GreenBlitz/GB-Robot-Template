package frc.robot.subsystems.funnel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record FunnelStuff(
	String logPath,
	String shooterDigitalInputLogPath,
	String ampDigitalInputLogPath,
	IMotor motor,
	InputSignal<Double> voltageSignal,
	InputSignal<Rotation2d> positionSignal,
	IDigitalInput shooterDigitalInput,
	IDigitalInput ampDigitalInput

) {

	public FunnelStuff(
		String logPath,
		IMotor motor,
		InputSignal<Double> voltageSignal,
		InputSignal<Rotation2d> positionSignal,
		IDigitalInput shooterDigitalInput,
		IDigitalInput ampDigitalInput
	) {
		this(
			logPath,
			logPath + "shooterDigitalInput",
			logPath + "ampDigitalInput",
			motor,
			voltageSignal,
			positionSignal,
			shooterDigitalInput,
			ampDigitalInput
		);
	}

}
