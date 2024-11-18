package frc.robot.subsystems.swerve.factories.gyro;

<<<<<<< HEAD
import frc.robot.hardware.gyro.maple.MapleGyro;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.swerve.GyroStuff;
import frc.utils.AngleUnit;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class SimulationGyroConstants {

	public static GyroSimulation generateGyroSimulation() {
		return GyroSimulation.createPigeon2();
	}

	public static GyroStuff generateGyroStuff(String logPath, GyroSimulation gyroSimulation) {
		return new GyroStuff(
			logPath,
			new MapleGyro(logPath, gyroSimulation),
			new SuppliedAngleSignal("yaw", () -> gyroSimulation.getGyroReading().getRotations(), AngleUnit.ROTATIONS)
		);
=======
import frc.robot.hardware.empties.EmptyGyro;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtils;

public class SimulationGyroConstants {

	protected static IGyro generateGyro(String logPath) {
		return new EmptyGyro(logPath);
	}

	protected static GyroSignals generateSignals() {
		return new GyroSignals(new AngleSignal("yaw", AngleUnit.DEGREES) {

			@Override
			protected TimedValue<Double> getNewValue() {
				return new TimedValue<>(0.0, TimeUtils.getCurrentTimeSeconds());
			}

		});
>>>>>>> core-swerve
	}

}
