package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.robot.vision.DetectedObjectType;
import frc.utils.filter.Filter;
import frc.utils.math.ToleranceMath;

import java.util.Arrays;
import java.util.function.Supplier;

public class LimelightFilters {

	public static Filter detectedObjectFilter(Limelight limelight) {
		return Filter.nonFilteringFilter();
	}

	public static Filter megaTag1Filter(Limelight limelight) {
		return Filter.nonFilteringFilter();
	}

	public static Filter megaTag2Filter(Limelight limelight) {
		return Filter.nonFilteringFilter();
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
