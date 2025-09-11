package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.*;
import frc.robot.hardware.interfaces.InputSignal;

public record IMUSignals(
	InputSignal<Rotation2d> pitchSignal,
	InputSignal<Rotation2d> rollSignal,
	InputSignal<Rotation2d> yawSignal,
	InputSignal<Rotation2d> angularVelocityRollSignal,
	InputSignal<Rotation2d> angularVelocityPitchSignal,
	InputSignal<Rotation2d> angularVelocityYawSignal,
	InputSignal<Double> accelerationXSignal,
	InputSignal<Double> accelerationYSignal,
	InputSignal<Double> accelerationZSignal
) {}
