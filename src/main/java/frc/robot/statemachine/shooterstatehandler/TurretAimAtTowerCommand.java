package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachine.ScoringHelpers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

public class TurretAimAtTowerCommand extends Command {

	private final Arm turret;
	private final Supplier<Translation2d> target;
	private final Supplier<Pose2d> robotPose;

	public TurretAimAtTowerCommand(Arm turret, Supplier<Translation2d> target, Supplier<Pose2d> robotPose) {
		this.turret = turret;
		this.target = target;
		this.robotPose = robotPose;
		addRequirements(turret);
	}

	@Override
	public void execute() {
		Supplier<Rotation2d> targetAngle = ShooterStateHandler.getRobotRelativeLookAtTowerAngleForTurret(target.get(), robotPose.get());
		if (ShooterStateHandler.isTurretMoveLegal(targetAngle, turret)) {
			turret.setTargetPosition(targetAngle.get());
			Logger.recordOutput(ShooterConstants.LOG_PATH + "/IsTurretGoingToPosition", true);
		} else {
			turret.stayInPlace();
			Logger.recordOutput(ShooterConstants.LOG_PATH + "/IsTurretGoingToPosition", false);
			Logger.recordOutput(TurretConstants.LOG_PATH + "/PositionTarget", targetAngle.get());
		}
		Logger.recordOutput(ShooterConstants.LOG_PATH + "/TurretOnRobot", new Pose2d(robotPose.get().getTranslation(), turret.getPosition()));
		Logger.recordOutput(ShooterConstants.LOG_PATH + "/ClosestTower", ScoringHelpers.getClosestTower(robotPose.get()).getTower());
	}

}
