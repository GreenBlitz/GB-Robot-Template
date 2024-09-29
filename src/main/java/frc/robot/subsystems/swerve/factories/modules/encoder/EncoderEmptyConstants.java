package frc.robot.subsystems.swerve.factories.modules.encoder;

import edu.wpi.first.hal.HALUtil;
import frc.robot.hardware.angleencoder.EmptyAngleEncoder;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.subsystems.swerve.modules.stuffs.EncoderStuff;
import frc.utils.AngleUnit;
import frc.utils.Conversions;

public class EncoderEmptyConstants {

	protected static EncoderStuff generateEncoderStuff(String logPath) {
		AngleSignal emptySignal = new AngleSignal("empty signal", AngleUnit.ROTATIONS) {

			@Override
			protected TimedValue<Double> getNewValue() {
				return new TimedValue<>(0.0, Conversions.microSecondsToSeconds(HALUtil.getFPGATime()));
			}

		};
		return new EncoderStuff(new EmptyAngleEncoder(logPath), emptySignal);
	}

}
