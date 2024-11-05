package frc.robot.hardware.gyro.maple;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.gyro.IGyro;
import frc.robot.hardware.signal.InputSignal;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.littletonrobotics.junction.Logger;

public class MapleGyro implements IGyro {

	private final GyroSimulation gyroSimulation;
	private final String logPath;

	public MapleGyro(String logPath, GyroSimulation gyroSimulation) {
		this.logPath = logPath;
		this.gyroSimulation = gyroSimulation;
	}

	@Override
	public void setYaw(Rotation2d yaw) {
		gyroSimulation.setRotation(yaw);
	}

	@Override
	public boolean isConnected() {
		return true;
	}

	@Override
	public void updateInputs(InputSignal<?>... inputSignals) {
		for (InputSignal<?> inputSignal : inputSignals) {
			Logger.processInputs(logPath, inputSignal);
		}
	}

}
