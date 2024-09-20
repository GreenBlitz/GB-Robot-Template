package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class DiscretizationInterpolator extends DoubleBilinearInterpolation {

	public static final double DEFAULT_FUDGE_FACTOR = 1;
	public DiscretizationInterpolator() {
		super();
	}

	@SafeVarargs
	public DiscretizationInterpolator(Pair<Translation2d, Double>... knownPoints) {
		super(knownPoints);
	}

	@SuppressWarnings("finally")
	@Override
	public double getInterpolatedValue(Translation2d query) {
		double returnValue = Double.NaN;
		try {
			returnValue = super.getInterpolatedValue(query);
		} catch (Exception e) {
			returnValue = Double.NaN;
			System.out.println(e);
		} finally {
			return Double.isNaN(returnValue) ? DEFAULT_FUDGE_FACTOR : Math.max(returnValue,DEFAULT_FUDGE_FACTOR);
		}

	}


}
