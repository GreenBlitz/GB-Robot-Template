package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class DriveDistanceCommand extends FunctionalCommand {

	private final double constantPower = .5;
	private final static double steerToleranceRadians = .01;

	public DriveDistanceCommand(ModuleAlon subsystem, Rotation2d drive, Rotation2d angle) {
		super(() -> subsystem.stop(),() -> subsystem.steerToPosition(angle.getRadians()),(b)->subsystem.stop(),() -> MathUtilBlitz.tolerance(angle.getRadians(), subsystem.getSteerAngle().getRadians(), steerToleranceRadians));
		int signOfPosMDrive = (int)Math.signum(subsystem.getLinearAngle().minus(angle).getRadians());
		addRequirements(subsystem);
		FunctionalCommand driveToPosition = new FunctionalCommand(()->{},() -> subsystem.linearStablePower(constantPower),(b)->subsystem.linearSetPower(0),()->(subsystem.getLinearAngle().minus(angle).times(signOfPosMDrive).getRadians()<0));
		this.andThen(driveToPosition);
	}

}
