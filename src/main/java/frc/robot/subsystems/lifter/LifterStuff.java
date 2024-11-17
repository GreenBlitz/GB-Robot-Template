package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;

public record LifterStuff(
	String logPath,
	ControllableMotor motor,
	double drumRadius,
	IDigitalInput limitSwitch,
	InputSignal<Rotation2d> positionSignal,
	InputSignal... otherSignals
) {}
