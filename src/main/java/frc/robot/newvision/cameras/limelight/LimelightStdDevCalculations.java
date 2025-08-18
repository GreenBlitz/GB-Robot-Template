package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.function.Supplier;

public class LimelightStdDevCalculations {

	public static final Matrix<N3, N1> DEFAULT_STD_DEVS = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0003, 0.0003, 0.003);

	public static Supplier<Matrix<N3, N1>> getMegaTag1StdDevsCalculation(
		Limelight limelight,
		Matrix<N3, N1> minStdDevs,
		Matrix<N3, N1> stdDevFactors
	) {
		return () -> averageTagDistanceParabola(
			limelight.getMT1RobotPoseEstimate().avgTagDist,
			minStdDevs,
			stdDevFactors
		);
	}

	public static Supplier<Matrix<N3, N1>> getMegaTag2StdDevsCalculation(
		Limelight limelight,
		Matrix<N3, N1> minStdDevs,
		Matrix<N3, N1> stdDevFactors
	) {
		return () -> averageTagDistanceParabola(
			limelight.getMT2RobotPoseEstimate().avgTagDist,
			minStdDevs,
			stdDevFactors
		);
	}

	private static Matrix<N3, N1> averageTagDistanceParabola(
		double averageTagDistance,
		Matrix<N3, N1> minStandardDeviations,
		Matrix<N3, N1> standardDeviationFactors
	) {
		double averageTagDistanceSquared = Math.pow(averageTagDistance, 2);
		return MatBuilder.fill(
			Nat.N3(),
			Nat.N1(),
			Math.max(minStandardDeviations.get(0, 0), standardDeviationFactors.get(0, 0) * averageTagDistanceSquared),
			Math.max(minStandardDeviations.get(1, 0), standardDeviationFactors.get(1, 0) * averageTagDistanceSquared),
			Math.max(minStandardDeviations.get(2, 0), standardDeviationFactors.get(2, 0) * averageTagDistanceSquared)
		);
	}

}
