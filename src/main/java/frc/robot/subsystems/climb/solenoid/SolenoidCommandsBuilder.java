package frc.robot.subsystems.climb.solenoid;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GBCommandsBuilder;

import java.util.function.DoubleSupplier;

public class SolenoidCommandsBuilder extends GBCommandsBuilder {

	private final Solenoid solenoid;

	public SolenoidCommandsBuilder(Solenoid solenoid) {
		super();
		this.solenoid = solenoid;
	}

	public Command setPower(double power) {
		return solenoid.asSubsystemCommand(new RunCommand(() -> solenoid.setPower(power)), "Set power to " + power);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return solenoid.asSubsystemCommand(new RunCommand(() -> solenoid.setPower(powerSupplier.getAsDouble())), "Set power by supplier");
	}

	public Command stop() {
		return solenoid.asSubsystemCommand(new RunCommand(solenoid::stop), "Stop");
	}

}
