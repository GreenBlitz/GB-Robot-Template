package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.signal.InputSignal;

public record LifterComponents(
	String logPath,
	ControllableMotor motor,
	double drumRadius,
	InputSignal<Rotation2d> positionSignal,
	InputSignal... otherSignals
) {}
