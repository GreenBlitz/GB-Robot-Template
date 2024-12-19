package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class StandardDeviations2d {

	private double xStandardDeviationsMeters;
	private double yStandardDeviationsMeters;
	private Rotation2d angleStandardDeviations;

	public StandardDeviations2d() {
		xStandardDeviationsMeters = 0;
		yStandardDeviationsMeters = 0;
		angleStandardDeviations = new Rotation2d();
	}

	public StandardDeviations2d(double[] stdDevs) {
		this.xStandardDeviationsMeters = stdDevs[0];
		this.yStandardDeviationsMeters = stdDevs[1];
		this.angleStandardDeviations = Rotation2d.fromRadians(stdDevs[2]);
	}

	public StandardDeviations2d(double xStandardDeviationsMeters, double yStandardDeviationsMeters, Rotation2d angleStandardDeviations) {
		this.xStandardDeviationsMeters = xStandardDeviationsMeters;
		this.yStandardDeviationsMeters = yStandardDeviationsMeters;
		this.angleStandardDeviations = angleStandardDeviations;
	}

	public double getXStandardDeviationsMeters() {
		return xStandardDeviationsMeters;
	}

	public void setXStandardDeviationsMeters(double xStandardDeviationsMeters) {
		this.xStandardDeviationsMeters = xStandardDeviationsMeters;
	}

	public double getYStandardDeviationsMeters() {
		return yStandardDeviationsMeters;
	}

	public void setYStandardDeviationsMeters(double yStandardDeviationsMeters) {
		this.yStandardDeviationsMeters = yStandardDeviationsMeters;
	}

	public Rotation2d getAngleStandardDeviations() {
		return angleStandardDeviations;
	}

	public void setAngleStandardDeviations(Rotation2d angleStandardDeviations) {
		this.angleStandardDeviations = angleStandardDeviations;
	}

	public StandardDeviations2d withAngleStandardDeviations(Rotation2d angleStandardDeviations) {
		this.angleStandardDeviations = angleStandardDeviations;
		return this;
	}

	public StandardDeviations2d withXStandardDeviations(double xStandardDeviationsMeters) {
		this.xStandardDeviationsMeters = xStandardDeviationsMeters;
		return this;
	}

	public StandardDeviations2d withYStandardDeviations(double yStandardDeviationsMeters) {
		this.yStandardDeviationsMeters = yStandardDeviationsMeters;
		return this;
	}

	public Matrix<N3, N1> getWPILibStandardDeviations() {
		return VecBuilder.fill(xStandardDeviationsMeters, yStandardDeviationsMeters, angleStandardDeviations.getRadians());
	}

	public double getDeviationByPoseIndex(int index) {
		return new double[] {xStandardDeviationsMeters, yStandardDeviationsMeters, thetaStandardDeviations.getRadians()}[index];
	}

}
