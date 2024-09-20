package frc.utils.interpolator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class DiscritizationInterpolator extends DoubleBilinearInterpolation {

	public static final double DEFAULT_FUDGE_FACTOR = 1;
	public DiscritizationInterpolator() {
		super();
	}

	@SafeVarargs
	public DiscritizationInterpolator(Pair<Translation2d, Double>... knownPoints) {
		super(knownPoints);
	}

	@SuppressWarnings("finally")
	@Override
	public double getInterpolatedValue(Translation2d query) {
		double returnValue = 0;
		try {
			returnValue = super.getInterpolatedValue(query);
		} catch (Exception e) {
			System.out.println(e);
		}finally {
			if(returnValue == 0 || Double.isNaN(returnValue) || returnValue < 1) {
				return DEFAULT_FUDGE_FACTOR;
			}
			return returnValue;
		}

	}


}
