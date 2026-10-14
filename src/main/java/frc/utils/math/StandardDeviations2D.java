package frc.utils.math;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

public record StandardDeviations2D(double xStandardDeviations, double yStandardDeviations, double angleStandardDeviations) {

	public StandardDeviations2D() {
		this(0, 0, 0);
	}

	public StandardDeviations2D(double standardDeviation) {
		this(standardDeviation, standardDeviation, standardDeviation);
	}

	public StandardDeviations2D(StandardDeviations2D standardDeviations2D) {
		this(standardDeviations2D.xStandardDeviations, standardDeviations2D.yStandardDeviations, standardDeviations2D.angleStandardDeviations);
	}

	public Matrix<N3, N1> asColumnVector() {
		return VecBuilder.fill(xStandardDeviations, yStandardDeviations, angleStandardDeviations);
	}

	public double[] asPoseArray() {
		return new double[] {xStandardDeviations, yStandardDeviations, angleStandardDeviations};
	}

}
