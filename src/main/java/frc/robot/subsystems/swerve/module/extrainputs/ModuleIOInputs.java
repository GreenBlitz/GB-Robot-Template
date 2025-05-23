package frc.robot.subsystems.swerve.module.extrainputs;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleIOInputs {

	public ModuleIOData data = new ModuleIOData(0, 0, 0, 0, 0, 0, 0, 0, 0);

	public record ModuleIOData(
		double drivePositionRads,
		double driveVelocityRadPerSec,
		double driveTorqueCurrentAmps,
		double driveAppliedVolts,
		double turnAbsolutePositionRads,
		double turnPositionRads,
		double turnVelocityRadPerSec,
		double turnTorqueCurrentAmps,
		double turnAppliedVolts
	) {}

}
