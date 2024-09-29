package frc.robot.subsystems.solenoid;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.DoubleSupplier;

public class SolenoidCommandBuilder {
	
	private final Solenoid solenoid;
	
	public SolenoidCommandBuilder(Solenoid solenoid) {
		this.solenoid = solenoid;
	}
	
	public Command setPower(double power) {
		return new FunctionalCommand(
				() -> {},
				() -> solenoid.setPower(power),
				interrupted -> solenoid.stop(),
				() -> false
		);
	}
	
	public Command setPower(DoubleSupplier powerSupplier) {
		return new FunctionalCommand(
				() -> {},
				() -> solenoid.setPower(powerSupplier.getAsDouble()),
				interrupted -> solenoid.stop(),
				() -> false
		);
	}
	
	public Command stop() {
		return new InstantCommand(solenoid::stop, solenoid);
	}
	
}
