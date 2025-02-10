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

	private double peakVelocityMetersPerSecond;
	private double characterizationStartingTimeSeconds;
	private double peakVelocityTimeSeconds;

	public MaxVelocityAccelerationCharacterization(Swerve swerve, Consumer<Double> openLoopDriveByPower) {
		this.swerve = swerve;
		this.openLoopDriveByPower = openLoopDriveByPower;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		peakVelocityMetersPerSecond = 0;
		peakVelocityTimeSeconds = 0;
		characterizationStartingTimeSeconds = TimeUtil.getCurrentTimeSeconds();
	}

	@Override
	public void execute() {
		openLoopDriveByPower.accept(MAX_POWER);

		double currentVelocityMagnitudeMetersPerSecond = SwerveMath.getDriveMagnitude(swerve.getRobotRelativeVelocity());
		if (Math.abs(currentVelocityMagnitudeMetersPerSecond) > Math.abs(peakVelocityMetersPerSecond)) {
			peakVelocityMetersPerSecond = currentVelocityMagnitudeMetersPerSecond;
			peakVelocityTimeSeconds = TimeUtil.getCurrentTimeSeconds();
		}
	}

	@Override
	public void end(boolean interrupted) {
		double timeToReachPeakVelocitySeconds = peakVelocityTimeSeconds - characterizationStartingTimeSeconds;

		Logger.recordOutput(LOG_PATH + "/PeakVelocityMetersPerSecond", peakVelocityMetersPerSecond);
		Logger.recordOutput(LOG_PATH + "/TimeToReachPeakVelocitySeconds", timeToReachPeakVelocitySeconds);
		Logger.recordOutput(LOG_PATH + "/PeakAccelerationMetersPerSecondSquared", peakVelocityTimeSeconds / timeToReachPeakVelocitySeconds);
	}

}
