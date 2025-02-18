package frc.robot.subsystems.climb.lifter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class LifterCommandsBuilder {

	private final Lifter lifter;

	public LifterCommandsBuilder(Lifter lifter) {
		this.lifter = lifter;
	}

	public Command setPower(double power) {
		return lifter.asSubsystemCommand(new RunCommand(() -> lifter.setPower(power)), "Set power to " + power);
	}

	public Command stop() {
		return lifter.asSubsystemCommand(new RunCommand(lifter::stop), "Stop");
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return lifter.asSubsystemCommand(new RunCommand(() -> lifter.setPower(powerSupplier.getAsDouble())), "Set power by supplier");
	}

}
