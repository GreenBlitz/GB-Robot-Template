package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.LinkedList;
import java.util.List;

import static frc.utils.interpolator.InterpolationUtils.*;

public class DoubleBilinearInterpolation {

	private final List<Pair<Translation2d, Double>> knowPoints;

	public DoubleBilinearInterpolation() {
		this.knowPoints = new LinkedList<>();
	}

	@SafeVarargs
	public DoubleBilinearInterpolation(Pair<Translation2d, Double>... points) {
		this.knowPoints = new LinkedList<>();
		for (Pair<Translation2d, Double> point : points) {
			add(point);
		}
	}

	public double getInterpolatedValue(Translation2d query) {
		return bilinearInterpolate(query, knowPoints.toArray(Pair[]::new));
	}

	public void add(Pair<Translation2d, Double> point) {
		knowPoints.add(point);
	}


}
