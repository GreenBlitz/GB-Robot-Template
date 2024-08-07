package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.gyro.replay.EmptySwerveGyro;

public class GyroFactory {

	public static ISwerveGyro create(SwerveName swerveName) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL ->
				new Pigeon2Gyro(IDs.PIGEON_2_DEVICE_ID, RealGyroConstants.PIGEON_2_CONFIGURATION, swerveName.getLogPath());
			case SIMULATION, REPLAY -> new EmptySwerveGyro();
		};
	}

}
