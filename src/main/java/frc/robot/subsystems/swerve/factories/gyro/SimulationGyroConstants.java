package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.hardware.gyro.maple.MapleGyro;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.swerve.GyroStuff;
import frc.utils.AngleUnit;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import java.util.function.Supplier;

public class SimulationGyroConstants {

	public static Supplier<GyroSimulation> generateGyroSimulation() {
		return GyroSimulation.getPigeon2();
	}

	public static GyroStuff generateGyroStuff(String logPath, GyroSimulation gyroSimulation) {
		return new GyroStuff(
			logPath,
			new MapleGyro(logPath, gyroSimulation),
			new SuppliedAngleSignal("yaw", () -> gyroSimulation.getGyroReading().getRotations(), AngleUnit.ROTATIONS)
		);
	}

}
