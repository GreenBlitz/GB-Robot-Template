package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.hardware.angleencoder.EmptyAngleEncoder;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.cansparkmax.SparkMaxAngleSignal;
import frc.robot.subsystems.swerve.modules.stuffs.EncoderStuff;
import frc.utils.AngleUnit;

public class EncoderEmptyConstants {

	protected static EncoderStuff generateEncoderStuff(String logPath) {
		AngleSignal emptySignal = new SparkMaxAngleSignal("empty signal", () -> 0.0, AngleUnit.ROTATIONS);
		return new EncoderStuff(new EmptyAngleEncoder(logPath), emptySignal);
	}

}
