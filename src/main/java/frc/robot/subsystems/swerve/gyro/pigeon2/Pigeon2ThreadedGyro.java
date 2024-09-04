package frc.robot.subsystems.swerve.gyro.pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.gyro.GyroThreadInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.IThreadedGyro;
import frc.robot.subsystems.swerve.phoenix6signalsthread.Phoenix6SignalsThread;

import java.util.Queue;

public class Pigeon2ThreadedGyro extends Pigeon2Gyro implements IThreadedGyro {

	private final Queue<Double> yawQueue, timestampQueue;

	public Pigeon2ThreadedGyro(Pigeon2ConfigObject pigeon2ConfigObject) {
		super(pigeon2ConfigObject.getGyro(), pigeon2ConfigObject.getYawSignal());

		// todo - maybe latency
		this.yawQueue = Phoenix6SignalsThread.getInstance().registerRegularSignal(gyro, yawSignal);
		this.timestampQueue = Phoenix6SignalsThread.getInstance().getTimestampQueue();
	}

	@Override
	public void updateInputs(GyroThreadInputsAutoLogged inputs) {
		inputs.yawOdometrySamples = yawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
		inputs.timestampOdometrySamples = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
		yawQueue.clear();
		timestampQueue.clear();
	}

}
