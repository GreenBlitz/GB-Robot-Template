package frc.robot.subsystems.elevator.records;

import frc.robot.hardware.interfaces.ControllableMotor;

public record ElevatorMotorStuff(ControllableMotor motor, ElevatorRequests requests, ElevatorSignals signals) {}