package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.LinkedList;
import java.util.List;

public class DoubleBilinearInterpolation {

	private final List<Pair<Translation2d, Double>> knownPoints;

	public DoubleBilinearInterpolation() {
		this.knownPoints = new LinkedList<>();
	}

	@SafeVarargs
	public DoubleBilinearInterpolation(Pair<Translation2d, Double>... points) {
		this.knownPoints = new LinkedList<>();
		for (Pair<Translation2d, Double> point : points) {
			add(point);
		}
	}

	public double getInterpolatedValue(Translation2d query) throws Exception {
		return InterpolationUtils.bilinearInterpolate(query, knownPoints.toArray(Pair[]::new));
	}

	public void add(Pair<Translation2d, Double> point) {
		knownPoints.add(point);
	}


}
