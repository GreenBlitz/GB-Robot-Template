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

	@SafeVarargs
	public static double bilinearInterpolate(Translation2d query, Pair<Translation2d, Double>... knownPoints) {
		if (knownPoints.length < 4) {
			throw new IllegalArgumentException("At least 4 points and values are required.");
		}

		if (!isQueryPointInBoundingBox(query, knownPoints)) {
			throw new IllegalArgumentException("Query point is outside the bounding rectangle of the given points.");
		}

		Pair<Translation2d, Double>[] boundingBox = createBoundingBox(query, knownPoints);
		Pair<Translation2d, Double> bottomLeft = boundingBox[0];
		Pair<Translation2d, Double> bottomRight = boundingBox[1];
		Pair<Translation2d, Double> topLeft = boundingBox[2];
		Pair<Translation2d, Double> topRight = boundingBox[3];

		if (isAllPointsExists(bottomLeft, bottomRight, topLeft, topRight)) {
			throw new IllegalStateException("Bounding rectangle is not properly formed by given points.");
		}
		return biLinearInterpolate(bottomLeft, bottomRight, topLeft, topRight, query);
	}


}
