package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.constants.MathConstants;
import org.littletonrobotics.junction.Logger;

public class SimulationManager {

	private final String logPath;
	private final Robot robot;
    public static final double TURRET_DISTANCE_FROM_ROBOT_ON_X_AXIS = 0.25;

	public SimulationManager(String logPath, Robot robot) {
		this.logPath = logPath;
		this.robot = robot;
	}

	public void logPoses() {
		logIntakePosition3d();
		logTurretPosition3d();
		logHoodPosition3d();
	}

	private void logIntakePosition3d() {
		Logger.recordOutput(logPath + "/Intake", getIntakePosition3d(robot.getFourBar().getPosition()));
	}

	public void logTurretPosition3d() {
		Logger.recordOutput(logPath + "/Turret", getTurretPosition3d(robot.getTurret().getPosition()));
	}

	public void logHoodPosition3d() {
		Logger.recordOutput(logPath + "/Hood", getHoodPosition3d(robot.getHood().getPosition()));
	}

	public Pose3d getIntakePosition3d(Rotation2d intakePosition) {
		return new Pose3d(
			new Translation3d(0.0, 0.0, 0.0),
			new Rotation3d(intakePosition.minus(Rotation2d.fromDegrees(40)).getRadians(), 0.0, MathConstants.QUARTER_CIRCLE.getRadians())
		);
	}

	public Pose3d getTurretPosition3d(Rotation2d turretPosition) {
		return new Pose3d(
			new Translation3d(TURRET_DISTANCE_FROM_ROBOT_ON_X_AXIS, 0, 0.0),
			new Rotation3d(0.0, 0.0, turretPosition.getRadians() + MathConstants.QUARTER_CIRCLE.getRadians())
		);
	}

	public Pose3d getHoodPosition3d(Rotation2d hoodPosition) {
		return new Pose3d(
			new Translation3d(0.25, 0.0, 0.4),
			new Rotation3d(
				Rotation2d.fromDegrees(50.0).minus(hoodPosition).getRadians(),
				0.0,
				robot.getTurret().getPosition().getRadians() + MathConstants.QUARTER_CIRCLE.getRadians()
			)
		);
	}

}
