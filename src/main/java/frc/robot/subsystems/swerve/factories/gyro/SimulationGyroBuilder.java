package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.hardware.empties.EmptyGyro;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.utils.math.AngleUnit;
import frc.utils.time.TimeUtils;

class SimulationGyroBuilder {

	protected static IGyro buildGyro(String logPath) {
		return new EmptyGyro(logPath);
	}

	protected static GyroSignals buildSignals() {
		return new GyroSignals(new AngleSignal("yaw", AngleUnit.DEGREES) {

			@Override
			protected TimedValue<Double> getNewValue() {
				return new TimedValue<>(0.0, TimeUtils.getCurrentTimeSeconds());
			}

		});
	}

}
