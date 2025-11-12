package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GBCommandsBuilder;

import java.util.Set;
import java.util.function.Supplier;

public class RollerCommandsBuilder extends GBCommandsBuilder {

	private final Roller roller;

	RollerCommandsBuilder(Roller roller) {
		super();
		this.roller = roller;
		roller.setDefaultCommand(stop());
	}

	public Command setVoltage(double voltage) {
		return roller.asSubsystemCommand(new RunCommand(() -> roller.setVoltage(voltage)), "set roller voltage to " + voltage);
	}

	public Command stop() {
		return roller.asSubsystemCommand(new RunCommand(() -> roller.stop()), "stop roller");
	}

	public Command setBrake(boolean brake) {
		return roller.asSubsystemCommand(new InstantCommand(() -> roller.setBrake(brake)), "set brake");
	}

	public Command setPower(Supplier<Double> supplier) {
		return roller.asSubsystemCommand(new RunCommand(() -> roller.setPower(supplier.get())), "set power with supplier");
	}

	public Command setPower(Double power) {
		return roller.asSubsystemCommand(new RunCommand(() -> roller.setPower(power)), "set power to " + power);
	}

	public Command rollRotations(double rotations) {
		Rotation2d targetPosition = Rotation2d.fromRotations(rotations + roller.getPositio.getRotations());
		return roller.asSubsystemCommand(
			new DeferredCommand(() -> new RunCommand(() -> roller.goToPosition(targetPosition)), Set.of(roller)),
			"rollRotations " + rotations
		);
	}

}
