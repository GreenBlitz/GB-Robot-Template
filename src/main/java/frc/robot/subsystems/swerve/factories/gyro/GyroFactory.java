package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.gyro.EmptyThreadedGyro;
import frc.robot.subsystems.swerve.gyro.IThreadedGyro;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2ConfigObject;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2ThreadedGyro;

public class GyroFactory {

	public static IThreadedGyro create(SwerveName swerveName) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL ->
				new Pigeon2ThreadedGyro(
					new Pigeon2ConfigObject(IDs.PIGEON_2_DEVICE_ID, RealGyroConstants.generateGyroConfig(), swerveName.getLogPath())
				);
			case SIMULATION -> new EmptyThreadedGyro();
		};
	}

}
