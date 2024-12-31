package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record StandardDeviations2D(double xStandardDeviations, double yStandardDeviations, double angleStandardDeviations) {

	public StandardDeviations2D() {
		this(0, 0, 0);
	}

	public StandardDeviations2D withAngleStandardDeviations(double angleStandardDeviations) {
		return new StandardDeviations2D(this.xStandardDeviations, this.yStandardDeviations, angleStandardDeviations);
	}

	public StandardDeviations2D withXStandardDeviations(double xStandardDeviationsMeters) {
		return new StandardDeviations2D(xStandardDeviationsMeters, this.yStandardDeviations, this.angleStandardDeviations);
	}

	public StandardDeviations2D withYStandardDeviations(double yStandardDeviationsMeters) {
		return new StandardDeviations2D(this.xStandardDeviations, yStandardDeviationsMeters, this.angleStandardDeviations);
	}

	public Matrix<N3, N1> getWPILibStandardDeviations() {
		return VecBuilder.fill(xStandardDeviations, yStandardDeviations, angleStandardDeviations);
	}

}
