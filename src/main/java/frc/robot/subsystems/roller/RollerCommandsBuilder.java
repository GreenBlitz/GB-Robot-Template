package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GBCommandsBuilder;
import frc.utils.utilcommands.InitExecuteCommand;

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


	public Command setPower(Supplier<Double> supplier) {
		return roller.asSubsystemCommand(new RunCommand(() -> roller.setPower(supplier.get())), "set power with supplier");
	}

	public Command setPower(Double power) {
		return roller.asSubsystemCommand(new RunCommand(() -> roller.setPower(power)), "set power to " + power);
	}

	public Command rollRotationsAtVoltageForwards(double rotations, double voltage) {
		double finalVoltage = Math.abs(voltage);
		return roller.asSubsystemCommand(
			new DeferredCommand(
				() -> new InitExecuteCommand(
					() -> roller.updateTargetPosition(Rotation2d.fromRotations(rotations + roller.getPosition().getRotations())),
					() -> roller.setVoltage(finalVoltage)
				).until(roller.isPastPosition(roller.getTargetPosition())),
				Set.of(roller)
			),
			"Roll " + rotations + " rotations"
		);
	}

	public Command rollRotationsAtVoltageBackwards(double rotations, double voltage) {
		double finalVoltage = -Math.abs(voltage);
		double finalRotations = -Math.abs(rotations);
		return roller.asSubsystemCommand(
			new DeferredCommand(
				() -> new InitExecuteCommand(
					() -> roller.updateTargetPosition(Rotation2d.fromRotations(roller.getPosition().getRotations() - finalRotations)),
					() -> roller.setVoltage(finalVoltage)
				).until(roller.isPastPosition(roller.getTargetPosition())),
				Set.of(roller)
			),
			"Roll " + rotations + " rotations backwards"

		);
	}

}

