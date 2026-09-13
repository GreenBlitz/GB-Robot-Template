package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class DriveDistanceCommand extends FunctionalCommand {

	private final double constantPower = .5;
	private final static double steerToleranceRadians = .01;

	public DriveDistanceCommand(ModuleAlon subsystem, Rotation2d drive, Rotation2d angle) {
		super(() -> subsystem.stop(),() -> subsystem.steerToPosition(angle.getRadians()),(b)->subsystem.stop(),() -> MathUtilBlitz.tolerance(angle.getRadians(), subsystem.getSteerAngle().getRadians(), steerToleranceRadians));
		addRequirements(subsystem);
		int signOfDrive = (int)Math.signum(drive.getRadians());
		Rotation2d[] target = {null};
		FunctionalCommand driveToPosition = new FunctionalCommand(()->{target[0] = subsystem.getLinearAngle();},() -> subsystem.linearStablePower(constantPower),(b)->subsystem.linearSetPower(0),()->(target[0].plus(drive).minus(subsystem.getLinearAngle()).times(signOfDrive).getRadians()<=0));
		this.andThen(driveToPosition);
	}

}
