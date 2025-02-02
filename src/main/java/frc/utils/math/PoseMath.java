package frc.utils.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Function;

public class PoseMath {

	public static Translation2d getRelativeTranslation(Translation2d relativeTo, Translation2d toRelative) {
		return toRelative.minus(relativeTo);
	}

	public static Translation2d getRelativeTranslation(Pose2d relativeTo, Translation2d toRelative) {
		return getRelativeTranslation(relativeTo.getTranslation(), toRelative).rotateBy(relativeTo.getRotation().unaryMinus());
	}

	public static <T extends Iterable<E>, E> double calculateStandardDeviations(T values, Function<E, Double> getValue) {
		int length = 0;
		double total = 0;
		for (E value : values) {
			length++;
			total += getValue.apply(value);
		}
		double mean = total / length;
		double deviationsTotal = 0;
		for (E value : values) {
			deviationsTotal += Math.pow(getValue.apply(value) - mean, 2);
		}
		return Math.sqrt(deviationsTotal / length);
	}

}
