package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class FlyWheelCommandBuilder {

	private final FlyWheel flyWheel;

	public FlyWheelCommandBuilder(FlyWheel flyWheel) {
		this.flyWheel = flyWheel;
		flyWheel.setDefaultCommand(stop());
	}

	public Command setVelocity(Rotation2d velocity) {
		return new RunCommand(() -> flyWheel.setVelocity(velocity), flyWheel);
	}

	public Command stop() {
		return new RunCommand(flyWheel::stop, flyWheel);
	}

	public Command setBrake() {
		return new RunCommand(() -> flyWheel.setBrake(true), flyWheel);
	}

}

