package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.vision.DetectedObjectType;
import frc.utils.Filter;
import frc.utils.math.ToleranceMath;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class LimelightFilters {

	public static Filter detectedObjectFilter(Limelight limelight) {
		return Filter.nonFilteringFilter();
	}

	public static Filter megaTag1Filter(
		Limelight limelight,
		Function<Double, Optional<Rotation2d>> wantedYawAtTimestamp,
		Supplier<Boolean> isYawCalibrated,
		Translation2d robotInFieldTolerance,
		Rotation2d yawAtAngleTolerance
	) {
		double timestamp = limelight.getMT1RawData().timestampSeconds();
		return MegaTagFilters.isRobotInField(() -> limelight.getMT1RawData().pose().getTranslation(), robotInFieldTolerance)
			.and(
				MegaTagFilters.doesYawExistAtTimestamp(timestamp, wantedYawAtTimestamp)
					.and(
						MegaTagFilters.isYawAtExpectedAngle(
							() -> limelight.getMT1RawData().pose().getRotation(),
							() -> wantedYawAtTimestamp.apply(timestamp).get(),
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
		Translation2d robotInFieldTolerance,
		Rotation2d yawAtAngleTolerance
	) {
		double timestamp = limelight.getMT2RawData().timestampSeconds();
		return MegaTagFilters.isRobotInField(() -> limelight.getMT2RawData().pose().getTranslation(), robotInFieldTolerance)
			.and(
				MegaTagFilters.doesYawExistAtTimestamp(timestamp, wantedYawAtTimestamp)
					.and(
						MegaTagFilters.isYawAtExpectedAngle(
							() -> limelight.getMT2RawData().pose().getRotation(),
							() -> wantedYawAtTimestamp.apply(timestamp).get(),
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

		private static Filter doesYawExistAtTimestamp(double timestamp, Function<Double, Optional<Rotation2d>> getYawAtTimestamp) {
			return () -> getYawAtTimestamp.apply(timestamp).isPresent();
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

		private static Filter isXInField(Supplier<Double> robotX, double xToleranceMeters) {
			return () -> ToleranceMath.isInRange(robotX.get(), 0, Field.LENGTH_METERS, xToleranceMeters);
		}

		private static Filter isYInField(Supplier<Double> robotY, double yToleranceMeters) {
			return () -> ToleranceMath.isInRange(robotY.get(), 0, Field.WIDTH_METERS, yToleranceMeters);
		}

		private static Filter isRobotInField(Supplier<Translation2d> robotTranslation, Translation2d tolerance) {
			return isXInField(() -> robotTranslation.get().getX(), tolerance.getX())
				.and(isYInField(() -> robotTranslation.get().getY(), tolerance.getY()));
		}

	}

}
