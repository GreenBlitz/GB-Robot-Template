package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.statemachine.ScoringHelpers;
import frc.robot.subsystems.arm.Arm;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

public class TurretAimAtTowerCommand extends Command {

	private final Arm turret;
	private final Supplier<Pose2d> robotPose;

	public TurretAimAtTowerCommand(Arm turret, Supplier<Pose2d> robotPose) {
		this.turret = turret;
		this.robotPose = robotPose;
		addRequirements(turret);
	}

	@Override
	public void execute() {
		Translation2d target = ScoringHelpers.getClosestTower(robotPose.get()).getPose().getTranslation();
		Rotation2d targetAngle = ShooterStateHandler.getRobotRelativeLookAtTowerAngleForTurret(target, robotPose.get());
		if (ShooterStateHandler.isTurretMoveLegal(targetAngle, turret)) {
			turret.setTargetPosition(targetAngle);
			Logger.recordOutput(ShooterConstants.LOG_PATH + "/IsTurretGoingToPosition", true);
		} else {
			turret.stayInPlace();
			Logger.recordOutput(ShooterConstants.LOG_PATH + "/IsTurretGoingToPosition", false);
		}
	}

}
