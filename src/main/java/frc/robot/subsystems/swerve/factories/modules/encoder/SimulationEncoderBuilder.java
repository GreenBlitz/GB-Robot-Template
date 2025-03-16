package frc.robot.subsystems.swerve.factories.modules.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.empties.EmptyAngleEncoder;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.subsystems.swerve.module.records.EncoderSignals;
import frc.utils.TimedValue;
import frc.utils.math.AngleUnit;
import frc.utils.time.TimeUtil;

class SimulationEncoderBuilder {

	static IAngleEncoder buildEncoder(String logPath) {
		return new EmptyAngleEncoder(logPath);
	}

	static EncoderSignals buildSignals() {
		return new EncoderSignals(new AngleSignal("yaw", AngleUnit.DEGREES) {

			@Override
			protected void updateValue(TimedValue<Rotation2d> timedValue) {
				timedValue.setValue(new Rotation2d());
				timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
			}

		});
	}

}
