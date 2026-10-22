package frc.robot.subsystems.tal;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class DriveDistanceCommand extends Command {

	public ModuleTal module;
	private final double angle;
	private final double targetRotations;
	private double startPosition;

	public DriveDistanceCommand(ModuleTal module, double angle, double targetRotations) {
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
		if (this.targetRotations > 0) {
			this.module.setDrivePower(ModuleTalConstants.defaultPower);
		} else {
			this.module.setDrivePower(-ModuleTalConstants.defaultPower);
		}
	}

	@Override
	public boolean isFinished() {
		double result = (Math.abs(this.startPosition + this.targetRotations)) - Math.abs(this.module.getDrivePosition());
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
