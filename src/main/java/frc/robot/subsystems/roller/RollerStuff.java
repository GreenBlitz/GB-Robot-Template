package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.IMotor;
import frc.robot.hardware.interfaces.InputSignal;

public record RollerStuff(
	String logPath,
	String digitalInputLogPath,
	IMotor motor,
	InputSignal<Double> voltageSignal,
	InputSignal<Rotation2d> positionSignal,
	IDigitalInput digitalInput
) {

	public RollerStuff(
		String logPath,
		IMotor motor,
		InputSignal<Double> voltageSignal,
		InputSignal<Rotation2d> positionSignal,
		IDigitalInput digitalInput
	) {
		this(logPath, logPath + "digitalInput", motor, voltageSignal, positionSignal, digitalInput);
	}

}
