package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class WPILibStandardDeviations {
	
	private double xStandardDeviationsMeters;
	private double yStandardDeviationsMeters;
	private Rotation2d thetaStandardDeviations;
	
	public WPILibStandardDeviations() {
		xStandardDeviationsMeters = 0;
		yStandardDeviationsMeters = 0;
		thetaStandardDeviations = new Rotation2d();
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
	
	public WPILibStandardDeviations withThetaStandardDeviations(Rotation2d thetaStandardDeviations){
		this.thetaStandardDeviations = thetaStandardDeviations;
		return this;
	}
	
	public WPILibStandardDeviations withXStandardDeviations(double xStandardDeviationsMeters){
		this.xStandardDeviationsMeters = xStandardDeviationsMeters;
		return this;
	}
	
	public WPILibStandardDeviations withYStandardDeviations(double yStandardDeviationsMeters){
		this.yStandardDeviationsMeters = yStandardDeviationsMeters;
		return this;
	}
	
	public Matrix<N3,N1> getWPILibStandardDeviations(){
		return VecBuilder.fill(
				xStandardDeviationsMeters,
				yStandardDeviationsMeters,
				thetaStandardDeviations.getRadians()
		);
	}
}
