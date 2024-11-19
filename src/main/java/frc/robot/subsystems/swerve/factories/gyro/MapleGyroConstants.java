package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.hardware.gyro.maple.MapleGyro;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.utils.AngleUnit;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class MapleGyroConstants {

	protected static IGyro generateGyro(String logPath) {
		return new MapleGyro(logPath, GyroSimulation.createPigeon2());
	}

	protected static GyroSignals generateSignals(MapleGyro mapleGyro) {
		return new GyroSignals(
			new SuppliedAngleSignal("yaw", () -> mapleGyro.getGyroSimulation().getGyroReading().getRotations(), AngleUnit.ROTATIONS)
		);
	}

}
