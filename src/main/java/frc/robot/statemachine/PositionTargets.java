package frc.robot.statemachine;

import frc.robot.Robot;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.subsystems.swerve.Swerve;

public class PositionTargets {

	private final IPoseEstimator poseEstimator;
	private final Swerve swerve;

	public PositionTargets(Robot robot) {
//        this.poseEstimator = robot.getPoseEstimator();
//        this.swerve = robot.getSwerve();
		this.poseEstimator = null;
		this.swerve = null;
	}

}
