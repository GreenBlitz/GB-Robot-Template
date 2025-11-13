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
) {

    public static ArmSignals getSignals(TalonFXMotor motor,int signalFrequency,BusChain busChain){
        Phoenix6AngleSignal velocity = Phoenix6SignalBuilder.build(motor.getDevice().getVelocity(), signalFrequency, AngleUnit.ROTATIONS, busChain);
        return new ArmSignals(
                Phoenix6SignalBuilder.build(motor.getDevice().getMotorVoltage(), signalFrequency, busChain),
                Phoenix6SignalBuilder.build(motor.getDevice().getStatorCurrent(), signalFrequency, busChain),
                velocity,
                Phoenix6SignalBuilder
                        .build(motor.getDevice().getPosition(), velocity, signalFrequency,AngleUnit.ROTATIONS,busChain)
        );


    }

}
