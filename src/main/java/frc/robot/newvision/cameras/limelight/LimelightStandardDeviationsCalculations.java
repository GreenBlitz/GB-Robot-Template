package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.utils.LimelightHelpers;

import java.util.function.Function;

public class LimelightStandardDeviationsCalculations {

	public static Function<LimelightHelpers.PoseEstimate, Matrix<N3, N1>> averageTagDistanceParabola(
		Matrix<N1, N3> minStandardDeviations,
		Matrix<N1, N3> standardDeviationFactors
	) {
		return (poseEstimate) -> {
			double averageTagDistanceSquared = Math.pow(poseEstimate.avgTagDist, 2);
			return MatBuilder.fill(
				Nat.N3(),
				Nat.N1(),
				Math.max(minStandardDeviations.get(0, 0), standardDeviationFactors.get(0, 0) * averageTagDistanceSquared),
				Math.max(minStandardDeviations.get(0, 1), standardDeviationFactors.get(0, 1) * averageTagDistanceSquared),
				Math.max(minStandardDeviations.get(0, 2), standardDeviationFactors.get(0, 2) * averageTagDistanceSquared)
			);
		};
	}

}
