package frc.robot.subsystems.swerve.factories.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.swerve.IMUSignals;
import frc.robot.subsystems.swerve.maplewrappers.MapleGyro;
import frc.utils.TimedValue;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtil;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class MapleIMUBuilder {

	public static IIMU buildIMU(String logPath, SwerveDriveSimulation swerveDriveSimulation) {
		return new MapleGyro(logPath + "/Maple", swerveDriveSimulation.getGyroSimulation());
	}

	public static IMUSignals buildSignals(MapleGyro mapleGyro) {
		return new IMUSignals(
			buildEmptyAngleSignal("roll"),
			buildEmptyAngleSignal("pitch"),
			new SuppliedAngleSignal("yaw", () -> mapleGyro.getGyroSimulation().getGyroReading().getRadians(), AngleUnit.RADIANS),
			buildEmptyAngleSignal("angularVelocityX"),
			buildEmptyAngleSignal("angularVelocityY"),
			new SuppliedAngleSignal(
				"yaw",
				() -> mapleGyro.getGyroSimulation().getMeasuredAngularVelocity().in(Units.RadiansPerSecond),
				AngleUnit.RADIANS
			),
			buildEmptyDoubleSignal("accelerationX"),
			buildEmptyDoubleSignal("accelerationY"),
			buildEmptyDoubleSignal("accelerationZ")
		);
	}

	private static AngleSignal buildEmptyAngleSignal(String name) {
		return new AngleSignal(name, AngleUnit.DEGREES) {

			@Override
			protected void updateValue(TimedValue<Rotation2d> timedValue) {
				timedValue.setValue(new Rotation2d());
				timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
			}

		};
	}

	private static DoubleSignal buildEmptyDoubleSignal(String name) {
		return new DoubleSignal(name) {

			@Override
			protected void updateValue(TimedValue<Double> timedValue) {
				timedValue.setValue(0.0);
				timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
			}

		};
	}


}
