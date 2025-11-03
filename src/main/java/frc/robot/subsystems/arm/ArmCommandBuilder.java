package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.statemachine.superstructure.TargetChecks;
import frc.robot.subsystems.GBCommandsBuilder;
import frc.utils.math.ToleranceMath;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ArmCommandBuilder extends GBCommandsBuilder {
    private final Arm arm;

    public ArmCommandBuilder(Arm arm) {
        this.arm = arm;
    }

    public Command stayInPlace() {
        return new InstantCommand(arm::stayInPlace, arm);
    }

    public Command setPower(double power) {
        return new RunCommand(() -> arm.setPower(power), arm);
    }

    public Command setPower(DoubleSupplier powerSupplier) {
        return arm.asSubsystemCommand(new RunCommand(() -> arm.setPower(powerSupplier.getAsDouble())), "Set power by supplier");
    }

    public Command setNeutralMode(boolean brake) {
        return new InstantCommand(()-> arm.setBrake(brake),arm);
    }

    public Command moveToPosition(Rotation2d target){
        return new RunCommand(() -> arm.setTargetPosition(target),arm);
    }

    public Command setVoltage(Double voltage){
        return new RunCommand(() -> arm.setVoltage(voltage),arm);
    }

}

