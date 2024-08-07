package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils.ModulePosition;

public enum RotateAxis {

	MIDDLE_OF_ROBOT(0, 0),
	FRONT_LEFT_MODULE(SwerveConstants.LOCATIONS[ModulePosition.FRONT_LEFT.getIndex()]),
	FRONT_RIGHT_MODULE(SwerveConstants.LOCATIONS[ModulePosition.FRONT_RIGHT.getIndex()]),
	BACK_LEFT_MODULE(SwerveConstants.LOCATIONS[ModulePosition.BACK_LEFT.getIndex()]),
	BACK_RIGHT_MODULE(SwerveConstants.LOCATIONS[ModulePosition.BACK_RIGHT.getIndex()]);

	private final Translation2d rotateAxis;

	RotateAxis(double x, double y) {
		this.rotateAxis = new Translation2d(x, y);
	}

	RotateAxis(Translation2d rotateAxis) {
		this.rotateAxis = new Translation2d(rotateAxis.getX(), rotateAxis.getY());
	}

	public Translation2d getRotateAxis() {
		return new Translation2d(rotateAxis.getX(), rotateAxis.getY());
	}


	public static RotateAxis getLeftFarRotateAxis() {
		return getFarRotateAxis(true);
	}

	public static RotateAxis getRightFarRotateAxis() {
		return getFarRotateAxis(false);
	}

	private static RotateAxis getFarRotateAxis(boolean isLeft) {
		Rotation2d currentAllianceAngle = Robot.swerve.getAllianceRelativeHeading();
		if (Math.abs(currentAllianceAngle.getDegrees()) <= MathConstants.EIGHTH_CIRCLE.getDegrees()) { // -45 <= x <= 45
			return isLeft ? FRONT_LEFT_MODULE : FRONT_RIGHT_MODULE;
		}
		if (Math.abs(currentAllianceAngle.getDegrees()) >= MathConstants.EIGHTH_CIRCLE.getDegrees() * 3) { // -135 - x - 135
			return isLeft ? BACK_RIGHT_MODULE : BACK_LEFT_MODULE;
		}
		if (currentAllianceAngle.getDegrees() > 0) { // 45 <= x <= 135
			return isLeft ? FRONT_RIGHT_MODULE : BACK_RIGHT_MODULE;
		}
		return isLeft ? BACK_LEFT_MODULE : FRONT_LEFT_MODULE; // -45 >= x >= -135
	}

}
