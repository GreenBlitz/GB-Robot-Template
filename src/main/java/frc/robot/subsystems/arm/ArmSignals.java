package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public record ArmSignals(
	InputSignal<Double> voltageSignal,
	InputSignal<Double> currentSignal,
	InputSignal<Rotation2d> velocitySignal,
	InputSignal<Rotation2d> positionSignal
) { }
