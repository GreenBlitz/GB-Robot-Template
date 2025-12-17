package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
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
		Rotation2d targetAngle = ShooterStateHandler.getRobotRelativeLookAtTowerAngleForTurret(target.get(), robotPose.get());
		if (ShooterStateHandler.isTurretMoveLegal(targetAngle, turret)) {
			turret.setTargetPosition(targetAngle);
			Logger.recordOutput(ShooterConstants.LOG_PATH + "/IsTurretGoingToPosition", true);
		} else {
			turret.stayInPlace();
			Logger.recordOutput(ShooterConstants.LOG_PATH + "/IsTurretGoingToPosition", false);
		}
	}

}
