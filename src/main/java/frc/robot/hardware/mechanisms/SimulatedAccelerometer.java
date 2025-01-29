package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.hardware.interfaces.IAccelerometer;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.time.TimeUtils;

import java.util.Random;
import java.util.function.Supplier;

public class SimulatedAccelerometer implements IAccelerometer {

	private final Supplier<ChassisSpeeds> robotSimulatedVelocityRelativeToRobot;
	private final Supplier<Double> fakedNoise;
	private final String logPath;

	private double accelerationX;
	private double accelerationY;
	private ChassisSpeeds previousRobotVelocities;
	private double lastUpdateTimestamp;

	public SimulatedAccelerometer(String logPath, Supplier<ChassisSpeeds> robotSimulatedVelocity, double fakedNoiseAmount) {
		Random randomNumbersGenerator = new Random();
		this.logPath = logPath;
		this.robotSimulatedVelocityRelativeToRobot = robotSimulatedVelocity;
		this.fakedNoise = () -> randomNumbersGenerator.nextGaussian() * fakedNoiseAmount;
		this.previousRobotVelocities = robotSimulatedVelocity.get();
		this.lastUpdateTimestamp = TimeUtils.getCurrentTimeSeconds();

		AlertManager.addAlert(
			new PeriodicAlert(
				Alert.AlertType.WARNING,
				"simulatedAccelerationIsNotUpdated",
				() -> lastUpdateTimestamp < TimeUtils.getCurrentTimeSeconds()
			)
		);
	}


	@Override
	public double getAccelerationX() {
		return accelerationX + fakedNoise.get();
	}

	@Override
	public double getAccelerationY() {
		return accelerationY + fakedNoise.get();
	}

	@Override
	public double getAccelerationZ() {
		return 0 + fakedNoise.get();
	}

	private void update() {
		double deltaTime = TimeUtils.getLatestCycleTimeSeconds();
		accelerationX = (previousRobotVelocities.vxMetersPerSecond - robotSimulatedVelocityRelativeToRobot.get().vxMetersPerSecond) / deltaTime;
		accelerationY = (previousRobotVelocities.vyMetersPerSecond - robotSimulatedVelocityRelativeToRobot.get().vyMetersPerSecond) / deltaTime;
		this.previousRobotVelocities = robotSimulatedVelocityRelativeToRobot.get();
		this.lastUpdateTimestamp = deltaTime;
		logAcceleration();
	}

	@Override
	public String getLogPath() {
		return logPath;
	}

}
