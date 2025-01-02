package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.InitExecuteCommand;

public class ArmCommandBuilder {
    public final Arm arm;

    public ArmCommandBuilder(Arm arm) {
        this.arm = arm;
    }

    public Command moveToPosition(Rotation2d angle) {
        return new FunctionalCommand(() -> arm.setTargetPosition(angle), () -> {
        }, finished -> arm.stayInPlace(), () -> false, arm).withName("Move to angle: " + angle);
    }

    public Command stayInPlace(){
        return new RunCommand(arm::stayInPlace, arm).withName("Stay in place");
    }

    public Command setVoltage(double voltage){
        return new RunCommand(() -> arm.setVoltage(voltage), arm).withName("Set voltage to: " + voltage);
    }


}
