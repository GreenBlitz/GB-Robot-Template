package frc.robot.subsystems.tal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class DriveDistanceCommand extends Command {

	public GBModuleTal module;
	private final Rotation2d angle;
	private final Rotation2d targetRotations;
	private Rotation2d startPosition;

	public DriveDistanceCommand(GBModuleTal module, Rotation2d angle, Rotation2d targetRotations) {
		this.module = module;
		this.angle = angle;
		this.targetRotations = targetRotations;
		this.addRequirements(this.module);
	}

	@Override
	public void initialize() {
		this.startPosition = module.getDrivePosition();

		this.module.stop();
	}

	@Override
	public void execute() {
		this.module.steerToAngle(this.angle);
		if (
			Math.abs(this.angle.getDegrees()) - Math.abs(this.module.getSteerAngle().getDegrees()) <= 1
				&& Math.abs(this.angle.getDegrees()) - Math.abs(this.module.getSteerAngle().getDegrees()) >= -1
		)
			if (this.targetRotations.getRotations() > 0) {
				this.module.setDrivePower(ModuleTalConstants.defaultPower);
			} else {
				this.module.setDrivePower(-ModuleTalConstants.defaultPower);
			}
	}

	@Override
	public boolean isFinished() {
		double result = (Math.abs(this.startPosition.getRotations() + this.targetRotations.getRotations()))
			- Math.abs(this.module.getDrivePosition().getRotations());
		Logger.recordOutput("drivedistanceResult", result);
		Logger.recordOutput("drivePoisition", this.module.getDrivePosition());
		Logger.recordOutput("startpoition", this.startPosition);

		return result <= 0;
	}

	@Override
	public void end(boolean Interrupted) {
		this.module.stop();
	}

}
