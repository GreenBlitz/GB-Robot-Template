package frc.robot.subsystems.moduleAlon;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class ModuleAlonMath {

	public static Rotation2d getSteerTargetAngleWithLinearFlipping(Rotation2d targetAngle, Rotation2d currentAngle) {
		return Rotation2d
			.fromRotations(getModInRange(targetAngle.getRotations(), currentAngle.getRotations() - 0.25, currentAngle.getRotations() + 0.25));
	}

	public static Rotation2d getSteerTargetAngleNoLinearFlipping(Rotation2d targetAngle, Rotation2d currentAngle) {
		return Rotation2d
			.fromRotations(getModInRange(targetAngle.getRotations(), currentAngle.getRotations() - 0.5, currentAngle.getRotations() + 0.5));
	}

	public static boolean shouldFlipLinear(Rotation2d targetSteerAngle, Rotation2d currentSteerAngle) {
		return Math.abs(((targetSteerAngle.getRotations() - currentSteerAngle.getRotations()) % 1.0) % 0.75) > 0.25;
	}

	public static double getModInRange(double x, double rangeStart, double rangeEnd) {
		return rangeStart + modulu(x - rangeStart, rangeEnd - rangeStart);
	}

	public static double modulu(double x, double y) {
		return ((x % y) + y) % y;
	}

	public static void driveSwerve(
		Consumer<Rotation2d> steerToPosition,
		Consumer<Double> linearPower,
		Supplier<Double> xAxis,
		Supplier<Double> yAxis,
		boolean withLinearFlipping,
		Supplier<Rotation2d> steerPosition
	) {
		double x = xAxis.get();
		double y = yAxis.get();
		Rotation2d angle = Rotation2d.fromRadians(Math.atan2(y, x));
		double magnitude = Math.hypot(x, y);
		if (withLinearFlipping) {
			linearPower.accept(magnitude * (shouldFlipLinear(angle, steerPosition.get()) ? -1 : 1));
			steerToPosition.accept(getSteerTargetAngleWithLinearFlipping(angle, steerPosition.get()));
		} else {
			linearPower.accept(magnitude);
			steerToPosition.accept(getSteerTargetAngleNoLinearFlipping(angle, steerPosition.get()));
		}
	}

}
