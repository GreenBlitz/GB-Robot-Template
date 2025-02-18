package frc.utils.calibration.swervecalibration.maxvelocityacceleration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

public class MaxVelocityAccelerationCharacterization extends Command {

	private static final String LOG_PATH = "Calibration/MaxVelocityAcceleration";
	private static final double MAX_POWER = 1.0;
	private static final double MAX_VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.02;
	private static final double MAX_VELOCITY_TOLERANCE_RADIANS_PER_SECOND = 0.02;

	private final Swerve swerve;
	private final Consumer<ChassisPowers> openLoopDriveByPower;
	private final VelocityType velocityType;
	private final double velocityTolerance;

	private double maxVelocity;
	private double characterizationStartingTimeSeconds;
	private double timeReachedMaxVelocitySeconds;

	public MaxVelocityAccelerationCharacterization(Swerve swerve, Consumer<ChassisPowers> openLoopDriveByPower, VelocityType velocityType) {
		this.swerve = swerve;
		this.openLoopDriveByPower = openLoopDriveByPower;
		this.velocityType = velocityType;
		this.velocityTolerance = velocityType == VelocityType.TRANSLATIONAL
			? MAX_VELOCITY_TOLERANCE_METERS_PER_SECOND
			: MAX_VELOCITY_TOLERANCE_RADIANS_PER_SECOND;

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		maxVelocity = 0;
		characterizationStartingTimeSeconds = TimeUtil.getCurrentTimeSeconds();
		timeReachedMaxVelocitySeconds = characterizationStartingTimeSeconds;
	}

	@Override
	public void execute() {
		ChassisPowers chassisPowers = switch (velocityType) {
			case TRANSLATIONAL -> new ChassisPowers(MAX_POWER, 0, 0);
			case ROTATIONAL -> new ChassisPowers(0, 0, MAX_POWER);
		};
		openLoopDriveByPower.accept(chassisPowers);

		double currentVelocity = switch (velocityType) {
			case TRANSLATIONAL -> SwerveMath.getDriveMagnitude(swerve.getRobotRelativeVelocity());
			case ROTATIONAL -> swerve.getRobotRelativeVelocity().omegaRadiansPerSecond;
		};
		if (Math.abs(currentVelocity) - Math.abs(maxVelocity) > velocityTolerance) {
			maxVelocity = currentVelocity;
			timeReachedMaxVelocitySeconds = TimeUtil.getCurrentTimeSeconds();
		}
	}

	@Override
	public void end(boolean interrupted) {
		double timeToReachMaxVelocitySeconds = timeReachedMaxVelocitySeconds - characterizationStartingTimeSeconds;

		String logPath = LOG_PATH + "/" + velocityType + "/";
		String unitName = velocityType == VelocityType.TRANSLATIONAL ? "Meters" : "Radians";
		Logger.recordOutput(logPath + "MaxVelocity" + unitName + "PerSecond", maxVelocity);
		Logger.recordOutput(logPath + "TimeToReachMaxVelocitySeconds", timeToReachMaxVelocitySeconds);
		Logger.recordOutput(logPath + "MaxAcceleration" + unitName + "PerSecondSquared", maxVelocity / timeToReachMaxVelocitySeconds);
	}

}
