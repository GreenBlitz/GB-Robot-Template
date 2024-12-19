package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class StandardDeviations2d {

	private double xStandardDeviationsMeters;
	private double yStandardDeviationsMeters;
	private Rotation2d thetaStandardDeviations;

	public StandardDeviations2d() {
		xStandardDeviationsMeters = 0;
		yStandardDeviationsMeters = 0;
		thetaStandardDeviations = new Rotation2d();
	}

	public StandardDeviations2d(double[] stdDevs) {
		this.xStandardDeviationsMeters = stdDevs[0];
		this.yStandardDeviationsMeters = stdDevs[1];
		this.thetaStandardDeviations = Rotation2d.fromRadians(stdDevs[2]);
	}

	public StandardDeviations2d(double xStandardDeviationsMeters, double yStandardDeviationsMeters, Rotation2d thetaStandardDeviations) {
		this.xStandardDeviationsMeters = xStandardDeviationsMeters;
		this.yStandardDeviationsMeters = yStandardDeviationsMeters;
		this.thetaStandardDeviations = thetaStandardDeviations;
	}

	public double getxStandardDeviationsMeters() {
		return xStandardDeviationsMeters;
	}

	public void setxStandardDeviationsMeters(double xStandardDeviationsMeters) {
		this.xStandardDeviationsMeters = xStandardDeviationsMeters;
	}

	public double getyStandardDeviationsMeters() {
		return yStandardDeviationsMeters;
	}

	public void setyStandardDeviationsMeters(double yStandardDeviationsMeters) {
		this.yStandardDeviationsMeters = yStandardDeviationsMeters;
	}

	public Rotation2d getThetaStandardDeviations() {
		return thetaStandardDeviations;
	}

	public void setThetaStandardDeviations(Rotation2d thetaStandardDeviations) {
		this.thetaStandardDeviations = thetaStandardDeviations;
	}

	public StandardDeviations2d withThetaStandardDeviations(Rotation2d thetaStandardDeviations) {
		this.thetaStandardDeviations = thetaStandardDeviations;
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
		return VecBuilder.fill(xStandardDeviationsMeters, yStandardDeviationsMeters, thetaStandardDeviations.getRadians());
	}

	public double getDeviationByPoseIndex(int index) {
		return new double[] {xStandardDeviationsMeters, yStandardDeviationsMeters, thetaStandardDeviations.getRadians()}[index];
	}

}
