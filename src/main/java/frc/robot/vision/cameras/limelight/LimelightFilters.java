package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.vision.DetectedObjectType;
import frc.utils.Filter;
import frc.utils.math.ToleranceMath;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class LimelightFilters {

	public static final double DEFAULT_IN_FIELD_TOLERANCE_METERS = 0.1;
	public static final Rotation2d DEFAULT_YAW_AT_ANGLE_TOLERANCE = Rotation2d.fromDegrees(10);

	public static Filter detectedObjectFilter(Limelight limelight) {
		return Filter.nonFilteringFilter();
	}

	public static Filter megaTag1Filter(
		Limelight limelight,
		Function<Double, Optional<Rotation2d>> wantedYawAtTimestamp,
		Supplier<Boolean> isYawCalibrated,
		double robotInFieldToleranceMeters,
		Rotation2d yawAtAngleTolerance
	) {
		return MegaTagFilters.isRobotInField(() -> limelight.getMT1RawData().pose().getTranslation(), robotInFieldToleranceMeters)
			.and(
				MegaTagFilters.doesYawExistAtTimestamp(() -> limelight.getMT1RawData().timestampSeconds(), wantedYawAtTimestamp)
					.implies(
						MegaTagFilters.isYawAtExpectedAngle(
							() -> limelight.getMT1RawData().pose().getRotation(),
							() -> wantedYawAtTimestamp.apply(limelight.getMT1RawData().timestampSeconds()).get(),
							isYawCalibrated,
							yawAtAngleTolerance
						)
					)
			);
	}

	public static Filter megaTag2Filter(
		Limelight limelight,
		Function<Double, Optional<Rotation2d>> wantedYawAtTimestamp,
		Supplier<Boolean> isYawCalibrated,
		double robotInFieldToleranceMeters,
		Rotation2d yawAtAngleTolerance
	) {
		return MegaTagFilters.isRobotInField(() -> limelight.getMT2RawData().pose().getTranslation(), robotInFieldToleranceMeters)
			.and(
				MegaTagFilters.doesYawExistAtTimestamp(() -> limelight.getMT2RawData().timestampSeconds(), wantedYawAtTimestamp)
					.implies(
						MegaTagFilters.isYawAtExpectedAngle(
							() -> limelight.getMT2RawData().pose().getRotation(),
							() -> wantedYawAtTimestamp.apply(limelight.getMT2RawData().timestampSeconds()).get(),
							isYawCalibrated,
							yawAtAngleTolerance
						)
					)
			)
			.and(MegaTagFilters.isYawNotZero(() -> limelight.getMT2RawData().pose().getRotation()));
	}

	private static class ObjectDetectionFilters {

		private static Filter onlyTheseTypes(Supplier<String> objectNameSupplier, DetectedObjectType... types) {
			return Filter.orAll(
				Arrays.stream(types)
					.map(objectType -> (Filter) () -> objectType.getName().equals(objectNameSupplier.get()))
					.toArray(Filter[]::new)
			);
		}

	}

	private static class MegaTagFilters {

		private static Filter doesYawExistAtTimestamp(DoubleSupplier timestamp, Function<Double, Optional<Rotation2d>> getYawAtTimestamp) {
			return () -> getYawAtTimestamp.apply(timestamp.getAsDouble()).isPresent();
		}

		private static Filter isYawAtExpectedAngle(
			Supplier<Rotation2d> cameraSuppliedRobotYaw,
			Supplier<Rotation2d> expectedYawSupplier,
			Supplier<Boolean> isExpectedYawCalibrated,
			Rotation2d expectedYawTolerance
		) {
			return () -> !isExpectedYawCalibrated.get()
				|| ToleranceMath.isNearWrapped(expectedYawSupplier.get(), cameraSuppliedRobotYaw.get(), expectedYawTolerance);
		}

		private static Filter isYawNotZero(Supplier<Rotation2d> robotYaw) {
			return () -> robotYaw.get().getRotations() != 0.0;
		}

		private static Filter isRobotInField(Supplier<Translation2d> robotTranslation, double toleranceMeters) {
			Translation2d currentRobotTranslation = robotTranslation.get();
			Rectangle2d field = new Rectangle2d(new Translation2d(0, 0), new Translation2d(Field.LENGTH_METERS, Field.WIDTH_METERS));
			return () -> field.getDistance(currentRobotTranslation) < toleranceMeters;
		}

	}

}
