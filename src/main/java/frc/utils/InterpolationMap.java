package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import java.util.Arrays;
import java.util.Comparator;
import java.util.Map;
import java.util.stream.Stream;

public class InterpolationMap<K, V> extends InterpolatingTreeMap<K, V> {

	public InterpolationMap(InverseInterpolator<K> inverseInterpolator, Interpolator<V> interpolator, Map<K, V> dataPoints) {
		super(inverseInterpolator, interpolator);
		put(dataPoints);
	}

	public InterpolationMap(InverseInterpolator<K> inverseInterpolator, Interpolator<V> interpolator) {
		super(inverseInterpolator, interpolator);
	}

	public InterpolationMap(InverseInterpolator<K> inverseInterpolator, Interpolator<V> interpolator, Comparator<K> comparator) {
		super(inverseInterpolator, interpolator, comparator);
	}

	public void put(Map<K, V> dataPoints) {
		dataPoints.forEach(super::put);
	}

	public Stream<V> get(K[] keys) {
		return Arrays.stream(keys).map(super::get);
	}

	public static Interpolator<Rotation2d> interpolatorForRotation2d() {
		return (startValue, endValue, t) -> Rotation2d
			.fromDegrees(Interpolator.forDouble().interpolate(startValue.getDegrees(), endValue.getDegrees(), t));
	}

	public static InverseInterpolator<Rotation2d> inverseInterpolatorForRotation2d() {
		return (startValue, endValue, t) -> InverseInterpolator.forDouble()
			.inverseInterpolate(startValue.getDegrees(), endValue.getDegrees(), t.getDegrees());
	}

}
