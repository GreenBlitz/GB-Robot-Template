package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

public class FlywheelCommands {
    private Flywheel flywheel;

    public FlywheelCommands(Flywheel flywheel) {
        this.flywheel = flywheel;
    }

    public Command setPowers(double rightPower, double leftPower){
        return new FunctionalCommand(
                () -> {},
                () -> flywheel.setPowers(rightPower, leftPower),
                (interrupted) -> flywheel.stop(),
                () -> false,
                flywheel
        );
    }

    public Command setVelocities(Rotation2d rightVelocity, Rotation2d leftVelocity) {
        return new FunctionalCommand(
                () -> {},
                () -> flywheel.setVelocities(rightVelocity, leftVelocity),
                (interrupted) -> flywheel.stop(),
                () -> false,
                flywheel
        );
    }

    public Command setPowersBySuppliers(Supplier<Rotation2d> rightVelocity, Supplier<Rotation2d> leftVelocity){
        return new FunctionalCommand(
                () -> {},
                () -> flywheel.setVelocities(rightVelocity.get(), leftVelocity.get()),
                (interrupted) -> {},
                () -> flywheel.isAtVelocities(rightVelocity.get() ,leftVelocity.get(), FlywheelConstants.TOLERANCE),
                flywheel
        );
    }
    public Command stop(){
        return new InstantCommand(() -> flywheel.stop(), flywheel);
    }
}
