package frc.robot.hardware.phoenix6.accelerometer;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import frc.robot.hardware.interfaces.IAccelerometer;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.time.TimeUtils;
import org.ejml.simple.SimpleMatrix;

import java.util.Random;
import java.util.function.Supplier;

public class simulatedAccelerometer extends GBSubsystem implements IAccelerometer {

	private final Supplier<ChassisSpeeds> robotSimulatedVelocityRelativeToRobot;
	private final Supplier<Double> noise;
	private double accelerationX;
	private double accelerationY;
	private ChassisSpeeds previousVelocity;

	public simulatedAccelerometer(String logPath, Supplier<ChassisSpeeds> robotSimulatedVelocityRelativeToRobot, double noiseAmount) {
		super(logPath);
		Random randomNumbersGenerator = new Random();

		this.robotSimulatedVelocityRelativeToRobot = robotSimulatedVelocityRelativeToRobot;
		this.noise = () -> randomNumbersGenerator.nextGaussian() * noiseAmount;
	}


	@Override
	public double getAccelerationMagnitude() {
		Vector<N3> accelerationVector = new Vector<>(
			new SimpleMatrix(new double[][] {{getAccelerationX(), getAccelerationY(), getAccelerationZ()}})
		);
		return accelerationVector.norm();
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

	@Override
	protected void subsystemPeriodic() {
		accelerationX = (previousVelocity.vxMetersPerSecond - robotSimulatedVelocityRelativeToRobot.get().vxMetersPerSecond)
			/ TimeUtils.getCurrentCycleTimeSeconds();
		accelerationY = (previousVelocity.vyMetersPerSecond - robotSimulatedVelocityRelativeToRobot.get().vyMetersPerSecond)
			/ TimeUtils.getCurrentCycleTimeSeconds();
		previousVelocity = robotSimulatedVelocityRelativeToRobot.get();
	}

}
