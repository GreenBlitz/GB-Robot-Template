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
	private static final double MAX_VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.02;

	private final Swerve swerve;
	private final Consumer<Double> openLoopDriveByPower;

	private double maxVelocityMetersPerSecond;
	private double characterizationStartingTimeSeconds;
	private double timeReachedMaxVelocitySeconds;

	public MaxVelocityAccelerationCharacterization(Swerve swerve, Consumer<Double> openLoopDriveByPower) {
		this.swerve = swerve;
		this.openLoopDriveByPower = openLoopDriveByPower;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		maxVelocityMetersPerSecond = 0;
		characterizationStartingTimeSeconds = TimeUtil.getCurrentTimeSeconds();
		timeReachedMaxVelocitySeconds = characterizationStartingTimeSeconds;
	}

	@Override
	public void execute() {
		openLoopDriveByPower.accept(MAX_POWER);

		double currentVelocityMagnitudeMetersPerSecond = SwerveMath.getDriveMagnitude(swerve.getRobotRelativeVelocity());
		if (
			Math.abs(currentVelocityMagnitudeMetersPerSecond) - Math.abs(maxVelocityMetersPerSecond) > MAX_VELOCITY_TOLERANCE_METERS_PER_SECOND
		) {
			maxVelocityMetersPerSecond = currentVelocityMagnitudeMetersPerSecond;
			timeReachedMaxVelocitySeconds = TimeUtil.getCurrentTimeSeconds();
		}
	}

	@Override
	public void end(boolean interrupted) {
		double timeToReachMaxVelocitySeconds = timeReachedMaxVelocitySeconds - characterizationStartingTimeSeconds;

		Logger.recordOutput(LOG_PATH + "/MaxVelocityMetersPerSecond", maxVelocityMetersPerSecond);
		Logger.recordOutput(LOG_PATH + "/TimeToReachMaxVelocitySeconds", timeToReachMaxVelocitySeconds);
		Logger.recordOutput(LOG_PATH + "/MaxAccelerationMetersPerSecondSquared", maxVelocityMetersPerSecond / timeToReachMaxVelocitySeconds);
	}

}
