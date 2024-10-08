package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.signal.InputSignal;

public record LifterComponents(
	String logPath,
	ControllableMotor motor,
	IDigitalInput limitSwitch,
	double drumRadius,
	IDigitalInput limitSwitch,
	InputSignal<Rotation2d> positionSignal,
	InputSignal... otherSignals
) {}
