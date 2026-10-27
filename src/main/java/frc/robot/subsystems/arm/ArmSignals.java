package frc.robot.subsystems.arm;

import org.wpilib.math.geometry.Angle2d;
import frc.robot.hardware.interfaces.InputSignal;

public record ArmSignals(
	InputSignal<Double> voltage,
	InputSignal<Double> statorCurrent,
	InputSignal<Double> torqueCurrent,
	InputSignal<Angle2d> velocity,
	InputSignal<Angle2d> position
) {}
