package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public record ArmSignals(
	InputSignal<Double> voltageSignal,
    InputSignal<Double> currentSignal,
	InputSignal<Rotation2d> velocitySignal,
    InputSignal<Rotation2d> positionSignal
) {

	public ArmSignals(TalonFXMotor arm, int signalFrequency, BusChain busChain) {
		this(
			Phoenix6SignalBuilder.build(arm.getDevice().getMotorVoltage(), signalFrequency, busChain),
			Phoenix6SignalBuilder.build(arm.getDevice().getStatorCurrent(), signalFrequency, busChain),
			Phoenix6SignalBuilder.build(arm.getDevice().getVelocity(), signalFrequency, AngleUnit.ROTATIONS, busChain),
			Phoenix6SignalBuilder
				.build(arm.getDevice().getPosition(), arm.getDevice().getVelocity(), signalFrequency, AngleUnit.ROTATIONS, busChain)
		);
	}

}
