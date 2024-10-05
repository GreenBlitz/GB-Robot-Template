package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class IntakeRollerCommandsBuilder {

	private IntakeRoller intakeRoller;

	public IntakeRollerCommandsBuilder(IntakeRoller intake) {
		this.intakeRoller = intake;
	}

	//@formatter:off
    public Command setPower(double power) {
        return new FunctionalCommand(
                () -> {},
                () -> intakeRoller.setPower(power),
                interrupted -> intakeRoller.stop(),
                () -> false,
                intakeRoller
        ).withName("set power: " + power);
    }

    public Command setPower(DoubleSupplier power) {
        return new FunctionalCommand(
                () -> {},
                () -> intakeRoller.setPower(power.getAsDouble()),
                interrupted -> intakeRoller.stop(),
                () -> false,
                intakeRoller
        ).withName("set power by supplier");
    }

    public Command stop() {
        return new RunCommand(intakeRoller::stop, intakeRoller).withName("stop");
    }

}

