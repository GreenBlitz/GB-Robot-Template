package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SimulationManager;
import frc.robot.statemachine.ScoringHelpers;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

public class TurretAimAtTowerCommand extends Command {

	private final Arm turret;
	private final Supplier<Pose2d> robotPose;
	private final String logPath;

	public TurretAimAtTowerCommand(Arm turret, Supplier<Pose2d> robotPose, String logPath) {
		this.turret = turret;
		this.robotPose = robotPose;
		this.logPath = logPath;
		addRequirements(turret);
	}

	@Override
	public void execute() {
		Pose2d turretOnField = new Pose2d(
			robotPose.get().getX() + robotPose.get().getRotation().getCos() * SimulationManager.TURRET_DISTANCE_FROM_ROBOT_ON_X_AXIS,
			robotPose.get().getY() + robotPose.get().getRotation().getSin() * SimulationManager.TURRET_DISTANCE_FROM_ROBOT_ON_X_AXIS,
			robotPose.get().getRotation()
		);
		Translation2d target = ScoringHelpers.getClosestTower(turretOnField).getPose().getTranslation();
		Rotation2d targetAngle = ShooterStateHandler.getRobotRelativeLookAtTowerAngleForTurret(target, turretOnField);
		if (ShooterStateHandler.isTurretMoveLegal(targetAngle, turret)) {
			turret.setTargetPosition(targetAngle);
			Logger.recordOutput(logPath + "/IsTurretGoingToPosition", true);
		} else {
			turret.stayInPlace();
			Logger.recordOutput(logPath + "/IsTurretGoingToPosition", false);
		}
	}

}
