package frc.robot.subsystems.arm;

import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public record ArmSignals(
	Phoenix6DoubleSignal voltageSignal,
	Phoenix6DoubleSignal currentSignal,
	Phoenix6AngleSignal velocitySignal,
	Phoenix6LatencySignal positionSignal
) {

	public ArmSignals(TalonFXMotor arm, int signalFrequency, BusChain busChain) {
		this(
			Phoenix6SignalBuilder.build(arm.getDevice().getMotorVoltage(), signalFrequency, busChain),
			Phoenix6SignalBuilder.build(arm.getDevice().getStatorCurrent(), signalFrequency, busChain),
			Phoenix6SignalBuilder.build(arm.getDevice().getVelocity(), signalFrequency, AngleUnit.ROTATIONS, busChain),
			Phoenix6SignalBuilder.build(
				arm.getDevice().getPosition(),
				arm.getDevice().getVelocity(),
				signalFrequency,
				AngleUnit.ROTATIONS,
				busChain
			)
		);
	}

}
