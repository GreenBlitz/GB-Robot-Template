package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class DriveDistanceCommand extends InstantCommand {

	private final double constantPower = .5;
	private final static double steerToleranceRadians = .01;

	public DriveDistanceCommand(ModuleAlon subsystem, Rotation2d drive, Rotation2d angle) {
		super(() -> subsystem.stop());
		addRequirements(subsystem);
		RunCommand steerToAngle = new RunCommand(() -> subsystem.steerToPosition(angle.getRadians()));
		steerToAngle.until(() -> MathUtilBlitz.tolerance(angle.getRadians(), subsystem.getSteerAngle().getRadians(), steerToleranceRadians));
		RunCommand driveToPosition = new RunCommand(() -> subsystem.linearToPosition(drive.getRotations(), constantPower));
		// driveToPosition.until()
	}

	@Override
	public void execute() {
		super.execute();
	}

}
