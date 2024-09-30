package frc.robot.subsystems.lifter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.function.DoubleSupplier;

public class LifterCommandsBuilder {

	private final Lifter lifter;

	public LifterCommandsBuilder(Lifter lifter) {
		this.lifter = lifter;
	}

	public Command setPower(double power) {
		return new FunctionalCommand(() -> {}, () -> lifter.setPower(power), (interrupted) -> lifter.stop(), () -> false, lifter)
			.withName("set power " + power);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return new FunctionalCommand(
			() -> {},
			() -> lifter.setPower(powerSupplier.getAsDouble()),
			(interrupted) -> lifter.stop(),
			() -> false,
			lifter
		).withName("set power by supplier");
	}

	public Command extend(double targetPosition) {
		return new FunctionalCommand(
			() -> {},
			() -> lifter.setPower(lifter.getLifterStuff().extendingPower()),
			(interrupted) -> lifter.stop(),
			() -> lifter.isAfter(targetPosition),
			lifter
		).withName("extend to position " + targetPosition);
	}

	public Command retract(double targetPosition) {
		return new FunctionalCommand(
			() -> {},
			() -> lifter.setPower(lifter.getLifterStuff().retractingPower()),
			(interrupted) -> lifter.stop(),
			() -> lifter.isBefore(targetPosition),
			lifter
		).withName("retract to position " + targetPosition);
	}


}
