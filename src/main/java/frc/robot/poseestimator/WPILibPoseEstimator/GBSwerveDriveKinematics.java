package frc.robot.poseestimator.WPILibPoseEstimator;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;

public class GBSwerveDriveKinematics extends SwerveDriveKinematics {

	private final SimpleMatrix inverseKinematics;
	private final SimpleMatrix forwardKinematics;

	private final int modulesNum;
	private final Translation2d[] moduleTranslations;
	private Rotation2d[] moduleHeadings;
	private Translation2d previousCenterOfRotation = Translation2d.kZero;

	public GBSwerveDriveKinematics(Translation2d... moduleTranslationsMeters) {
		if (moduleTranslationsMeters.length < 2) {
			throw new IllegalArgumentException("A swerve drive requires at least two modules");
		}
		modulesNum = moduleTranslationsMeters.length;
		moduleTranslations = Arrays.copyOf(moduleTranslationsMeters, modulesNum);
		moduleHeadings = new Rotation2d[modulesNum];
		Arrays.fill(moduleHeadings, Rotation2d.kZero);
		inverseKinematics = new SimpleMatrix(modulesNum * 2, 3);

		for (int i = 0; i < modulesNum; i++) {
			inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -moduleTranslations[i].getY());
			inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +moduleTranslations[i].getX());
		}
		forwardKinematics = inverseKinematics.pseudoInverse();

		MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
	}

	@Override
	public void resetHeadings(Rotation2d... moduleHeadings) {
		if (moduleHeadings.length != modulesNum) {
			throw new IllegalArgumentException(
				"Number of headings is not consistent with number of module locations provided in " + "constructor"
			);
		}
		this.moduleHeadings = Arrays.copyOf(moduleHeadings, modulesNum);
	}

	@Override
	public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
		var moduleStates = new SwerveModuleState[modulesNum];

		if (chassisSpeeds.vxMetersPerSecond == 0.0 && chassisSpeeds.vyMetersPerSecond == 0.0 && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
			for (int i = 0; i < modulesNum; i++) {
				moduleStates[i] = new SwerveModuleState(0.0, moduleHeadings[i]);
			}

			return moduleStates;
		}

		if (!centerOfRotationMeters.equals(previousCenterOfRotation)) {
			for (int i = 0; i < modulesNum; i++) {
				inverseKinematics.setRow(
					i * 2 + 0,
					0, /* Start Data */
					1,
					0,
					-moduleTranslations[i].getY() + centerOfRotationMeters.getY()
				);
				inverseKinematics.setRow(
					i * 2 + 1,
					0, /* Start Data */
					0,
					1,
					+moduleTranslations[i].getX() - centerOfRotationMeters.getX()
				);
			}
			previousCenterOfRotation = centerOfRotationMeters;
		}

		var chassisSpeedsVector = new SimpleMatrix(3, 1);
		chassisSpeedsVector
			.setColumn(0, 0, chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);

		var moduleStatesMatrix = inverseKinematics.mult(chassisSpeedsVector);

		for (int i = 0; i < modulesNum; i++) {
			double x = moduleStatesMatrix.get(i * 2, 0);
			double y = moduleStatesMatrix.get(i * 2 + 1, 0);

			double speed = Math.hypot(x, y);
			Rotation2d angle = speed > 1e-6 ? new Rotation2d(x, y) : moduleHeadings[i];

			moduleStates[i] = new SwerveModuleState(speed, angle);
			moduleHeadings[i] = angle;
		}

		return moduleStates;
	}

	@Override
	public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
		return toSwerveModuleStates(chassisSpeeds, Translation2d.kZero);
	}

	@Override
	public SwerveModuleState[] toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
		return toSwerveModuleStates(chassisSpeeds);
	}

	@Override
	public ChassisSpeeds toChassisSpeeds(SwerveModuleState... moduleStates) {
		if (moduleStates.length != modulesNum) {
			throw new IllegalArgumentException(
				"Number of modules is not consistent with number of module locations provided in " + "constructor"
			);
		}
		var moduleStatesMatrix = new SimpleMatrix(modulesNum * 2, 1);

		for (int i = 0; i < modulesNum; i++) {
			var module = moduleStates[i];
			moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
			moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
		}

		var chassisSpeedsVector = forwardKinematics.mult(moduleStatesMatrix);
		return new ChassisSpeeds(chassisSpeedsVector.get(0, 0), chassisSpeedsVector.get(1, 0), chassisSpeedsVector.get(2, 0));
	}

	@Override
	public Twist2d toTwist2d(SwerveModulePosition... moduleDeltas) {
		if (moduleDeltas.length != modulesNum) {
			throw new IllegalArgumentException(
				"Number of modules is not consistent with number of module locations provided in " + "constructor"
			);
		}
		var moduleDeltaMatrix = new SimpleMatrix(modulesNum * 2, 1);

		for (int i = 0; i < modulesNum; i++) {
			Pose2d poseDelta = calculateDeltaWheelPose(moduleDeltas[i]);
			moduleDeltaMatrix.set(i * 2, 0, poseDelta.getX());
			moduleDeltaMatrix.set(i * 2 + 1, poseDelta.getY());
		}

		var chassisDeltaVector = forwardKinematics.mult(moduleDeltaMatrix);
		return new Twist2d(chassisDeltaVector.get(0, 0), chassisDeltaVector.get(1, 0), chassisDeltaVector.get(2, 0));
	}

	@Override
	public Twist2d toTwist2d(SwerveModulePosition[] start, SwerveModulePosition[] end) {
		if (start.length != end.length) {
			throw new IllegalArgumentException("Inconsistent number of modules!");
		}
		var newPositions = new SwerveModulePosition[start.length];
		for (int i = 0; i < start.length; i++) {
			newPositions[i] = new SwerveModulePosition(end[i].distanceMeters - start[i].distanceMeters, end[i].angle);
		}
		return toTwist2d(newPositions);
	}
	
	public static Pose2d calculateDeltaWheelPose(SwerveModulePosition deltaWheelPosition) {
		Rotation2d deltaWheelOrientation = deltaWheelPosition.angle;
		if (deltaWheelOrientation.getRadians() == 0) {
			return new Pose2d(new Translation2d(deltaWheelPosition.distanceMeters, 0), deltaWheelOrientation);
		}
		
		double circleRadiusMeters = deltaWheelPosition.distanceMeters / deltaWheelOrientation.getRadians();
		Translation2d deltaWheelTranslation = new Translation2d(
				circleRadiusMeters * (deltaWheelOrientation.getCos() - 1),
				circleRadiusMeters * deltaWheelOrientation.getSin()
		);
		return new Pose2d(deltaWheelTranslation, deltaWheelOrientation);
	}

}
