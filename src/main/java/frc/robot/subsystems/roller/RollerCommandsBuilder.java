package frc.robot.subsystems.roller;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBCommandsBuilder;

import java.util.function.Supplier;

public class RollerCommandsBuilder extends GBCommandsBuilder {
    private final Roller roller;
    private final InputSignal<Double> voltageSignal;
    private final InputSignal<Double> currlentSignal;
    private final InputSignal<Rotation2d> latencySignal;
    private final InputSignal<Rotation2d> velocitySignal;

    RollerCommandsBuilder(Roller roller, InputSignal<Double> voltageSignal, InputSignal<Double> currlentSignal, InputSignal<Rotation2d> latencySignal, InputSignal<Rotation2d> velocitySignal){
        this.roller = roller;
        this.voltageSignal = voltageSignal;
        this.currlentSignal = currlentSignal;
        this.latencySignal = latencySignal;
        this.velocitySignal = velocitySignal;
        roller.setDefaultCommand(stop());
    }

    public Command setVoltage(double voltage){
        return roller.asSubsystemCommand(new RunCommand(() -> roller.setVoltage(voltage)),"set roller voltage");
    }

    public Command stop(){
        return  roller.asSubsystemCommand(new RunCommand(() -> roller.stop()),"stop roller");
    }

    public Command setBrake(boolean brake){
       return roller.asSubsystemCommand(new InstantCommand(() -> roller.setBrake(brake)),"set brake");
    }

    public Command setPowerWithSupplier(Supplier<Double> supplier){
        return roller.asSubsystemCommand(new RunCommand(() -> roller.setPower(supplier.get())),"set power with supplier");
    }

}
