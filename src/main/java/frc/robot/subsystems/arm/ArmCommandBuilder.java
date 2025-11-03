package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class ArmCommandBuilder  {
    public Arm arm;

    public ArmCommandBuilder(Arm arm) {
        this.arm = arm;
    }

    public Command stayInPlace() {
        return new InstantCommand(arm::stayInPlace, arm);
    }

    public Command setPower(double power) {
        return new RunCommand(() -> arm.setPower(power), arm);
    }

    public Command setBrake(boolean brake) {
        return new InstantCommand(()-> setBrake(brake),arm);
    }


}

