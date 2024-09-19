package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class DiscritizationInterpolator extends DoubleBilinearInterpolation {

	public DiscritizationInterpolator() {
		super();
	}

	@SafeVarargs
	public DiscritizationInterpolator(Pair<Translation2d, Double>... knownPoints) {
		super(knownPoints);
	}

	@Override
	public double getInterpolatedValue(Translation2d query) {
		double returnValue;
		try {
			returnValue = super.getInterpolatedValue(query);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		if (returnValue == 0 || Double.isNaN(returnValue) || returnValue < 1) {
			return 1;
		}
		return returnValue;
	}


}
