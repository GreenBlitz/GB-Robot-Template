package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.DoubleSupplier;

public class RollerCommandsBuilder {

	private final Roller roller;

	public RollerCommandsBuilder(Roller roller) {
		this.roller = roller;
	}

	//@formatter:on
	public Command moveByPower(double power) {
		return new FunctionalCommand(() -> roller.setPower(power), () -> {}, interrupted -> roller.stop(), () -> false, roller)
			.withName("Move by power: " + power);
	}

	public Command moveByPower(DoubleSupplier power) {
		return new FunctionalCommand(() -> {}, () -> roller.setPower(power.getAsDouble()), interrupted -> roller.stop(), () -> false, roller)
			.withName("Move by power supplier");
	}
	//@formatter:off

    public Command stop() {
        return new InstantCommand(roller::stop, roller).withName("Stop");
    }

}
