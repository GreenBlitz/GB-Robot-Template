package frc.robot.hardware.phoenix6.pigeon;

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
	private final Supplier<Double> noise;
	private final String logPath;

	private double accelerationX;
	private double accelerationY;
	private ChassisSpeeds previousRobotVelocities;
	private double lastUpdateTimestamp;

	public SimulatedAccelerometer(String logPath, Supplier<ChassisSpeeds> robotSimulatedVelocity, double noiseAmount) {
		Random randomNumbersGenerator = new Random();
		this.logPath = logPath;
		this.robotSimulatedVelocityRelativeToRobot = robotSimulatedVelocity;
		this.noise = () -> randomNumbersGenerator.nextGaussian() * noiseAmount;
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
		return accelerationX + noise.get();
	}

	@Override
	public double getAccelerationY() {
		return accelerationY + noise.get();
	}

	@Override
	public double getAccelerationZ() {
		return 0 + noise.get();
	}

	private void update() {
		double deltaTime = TimeUtils.getLatestCycleTimeSeconds();
		accelerationX = (previousRobotVelocities.vxMetersPerSecond - robotSimulatedVelocityRelativeToRobot.get().vxMetersPerSecond) / deltaTime;
		accelerationY = (previousRobotVelocities.vyMetersPerSecond - robotSimulatedVelocityRelativeToRobot.get().vyMetersPerSecond) / deltaTime;
		this.previousRobotVelocities = robotSimulatedVelocityRelativeToRobot.get();
		this.lastUpdateTimestamp = deltaTime;
		log();
	}

	public void log() {
		log(logPath);
	}

}
