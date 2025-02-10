package frc.utils.calibration.swervecalibration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

public class MaxVelocityAccelerationCharacterization extends Command {

	private static final String LOG_PATH = "Calibration/MaxVelocityAcceleration";
	private static final double MAX_POWER = 1.0;

	private final Swerve swerve;
	private final Consumer<Double> openLoopDriveByPower;

	private double maxVelocityMetersPerSecond;
	private double characterizationStartingTimeSeconds;
	private double maxVelocityTimeSeconds;

	public MaxVelocityAccelerationCharacterization(Swerve swerve, Consumer<Double> openLoopDriveByPower) {
		this.swerve = swerve;
		this.openLoopDriveByPower = openLoopDriveByPower;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		maxVelocityMetersPerSecond = 0;
		maxVelocityTimeSeconds = 0;
		characterizationStartingTimeSeconds = TimeUtil.getCurrentTimeSeconds();
	}

	@Override
	public void execute() {
		openLoopDriveByPower.accept(MAX_POWER);

		double currentVelocityMagnitudeMetersPerSecond = SwerveMath.getDriveMagnitude(swerve.getRobotRelativeVelocity());
		if (Math.abs(currentVelocityMagnitudeMetersPerSecond) > Math.abs(maxVelocityMetersPerSecond)) {
			maxVelocityMetersPerSecond = currentVelocityMagnitudeMetersPerSecond;
			maxVelocityTimeSeconds = TimeUtil.getCurrentTimeSeconds();
		}
	}

	@Override
	public void end(boolean interrupted) {
		double timeToReachMaxVelocitySeconds = maxVelocityTimeSeconds - characterizationStartingTimeSeconds;

		Logger.recordOutput(LOG_PATH + "/MaxVelocityMetersPerSecond", maxVelocityMetersPerSecond);
		Logger.recordOutput(LOG_PATH + "/TimeToReachMaxVelocitySeconds", timeToReachMaxVelocitySeconds);
		Logger.recordOutput(LOG_PATH + "/MaxAccelerationMetersPerSecondSquared", maxVelocityTimeSeconds / timeToReachMaxVelocitySeconds);
	}

}
