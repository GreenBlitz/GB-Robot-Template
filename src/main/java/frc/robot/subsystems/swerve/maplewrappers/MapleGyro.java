package frc.robot.subsystems.swerve.maplewrappers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.interfaces.InputSignal;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class MapleGyro implements IIMU {

	private final GyroSimulation gyroSimulation;

	public MapleGyro(GyroSimulation gyroSimulation) {
		this.gyroSimulation = gyroSimulation;
	}

	public GyroSimulation getGyroSimulation() {
		return gyroSimulation;
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
		// trust...
	}

}
