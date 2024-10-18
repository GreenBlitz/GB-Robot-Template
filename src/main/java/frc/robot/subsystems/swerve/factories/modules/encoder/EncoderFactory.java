package frc.robot.subsystems.swerve.factories.modules.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.hardware.angleencoder.EmptyAngleEncoder;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.stuffs.EncoderStuff;
import frc.utils.AngleUnit;

public class EncoderFactory {

	private static EncoderStuff createSwerveEncoder(String logPath, ModuleUtils.ModulePosition modulePosition) {
		return new EncoderStuff(new EmptyAngleEncoder(logPath), new SuppliedAngleSignal("angle, " + logPath,() -> 0.0, AngleUnit.ROTATIONS));
	}

	public static EncoderStuff create(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		String logPath = swerveType.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/Encoder/";
		return switch (swerveType) {
			case SWERVE -> createSwerveEncoder(logPath, modulePosition);
		};
	}

}
