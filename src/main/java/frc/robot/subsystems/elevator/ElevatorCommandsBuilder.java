package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class ElevatorCommandsBuilder {

    private final Elevator elevator;

    public ElevatorCommandsBuilder(Elevator elevator) {
        this.elevator = elevator;
    }

    public Command setPower(double power) {
        return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setPower(power), elevator), "Set Power To " + power);
    }

    public Command setPower(DoubleSupplier powerSupplier) {
        return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setPower(powerSupplier.getAsDouble())), "Set Power By Supplier");
    }

    public Command setVoltage(double voltage) {
        return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setVoltage(voltage)), "Set Voltage To " + voltage);
    }

    public Command setTargetPositionMeters(double targetPositionMeters) {
        return elevator.asSubsystemCommand(
                new RunCommand(() -> elevator.setTargetPositionMeters(targetPositionMeters)),
                "Set Target Position To " + targetPositionMeters + " Meters"
        );
    }

}