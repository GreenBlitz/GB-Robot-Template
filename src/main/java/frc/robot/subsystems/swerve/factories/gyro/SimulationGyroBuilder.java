package frc.robot.subsystems.swerve.factories.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.empties.EmptyGyro;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.utils.TimedValue;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtil;

class SimulationGyroBuilder {

	static IGyro buildGyro(String logPath) {
		return new EmptyGyro(logPath);
	}

	static GyroSignals buildSignals() {
		return new GyroSignals(new AngleSignal("yaw", AngleUnit.DEGREES) {

			@Override
			protected void updateValue(TimedValue<Rotation2d> timedValue) {
				timedValue.setValue(new Rotation2d());
				timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
			}

		});
	}

}
