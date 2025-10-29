package frc.robot.subsystems.climb.solenoid;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class SolenoidCommandsBuilder {

	private final Solenoid solenoid;
	private boolean isRunningIndependently;

	public SolenoidCommandsBuilder(Solenoid solenoid) {
		this.solenoid = solenoid;
		this.isRunningIndependently = false;
	}

	public boolean isRunningIndependently() {
		return isRunningIndependently;
	}

	public void setIsRunningIndependently(boolean isRunningIndependently) {
		this.isRunningIndependently = isRunningIndependently;
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
