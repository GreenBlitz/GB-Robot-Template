package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.hardware.empties.EmptyAngleEncoder;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.subsystems.swerve.module.records.EncoderSignals;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtils;

class SimulationEncoderBuilder {

	static IAngleEncoder buildEncoder(String logPath) {
		return new EmptyAngleEncoder(logPath);
	}

	static EncoderSignals buildSignals() {
		return new EncoderSignals(new AngleSignal("yaw", AngleUnit.DEGREES) {

			@Override
			protected TimedValue<Double> getNewValue() {
				return new TimedValue<>(0.0, TimeUtils.getCurrentTimeSeconds());
			}

		});
	}

}