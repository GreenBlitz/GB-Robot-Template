package frc.utils.gbmeth;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.Pose2dComponentsValue;
import frc.robot.poseestimator.Pose3dComponentsValue;
import frc.utils.AngleUnit;

import java.util.Collection;

public class GBVector3D extends GBVector<N3> {

	public GBVector3D(double[] data) {
		super(data);
	}

	public <E extends Num> GBVector3D(Vector<E> vector) {
		super(vector);
	}

	public GBVector3D(Collection<Double> data) {
		super(data);
	}

	public final GBVector3D cross(GBVector3D other) {
		return new GBVector3D(
			new double[] {
				this.get(1) * other.get(2) - this.get(2) * other.get(1),
				this.get(0) * other.get(2) - this.get(2) * other.get(0),
				this.get(0) * other.get(1) - this.get(1) * other.get(0)}
		);
	}

	public final void crossBy(GBVector3D other) {
		GBVector3D crossOutput = this.cross(other);
		this.factorOf = 0;
		this.data = crossOutput.data;
		this.isClone = false;
	}

	public final Pose2d toPose2d(AngleUnit angleUnit) {
		return new Pose2d(
			this.get(Pose2dComponentsValue.X_VALUE.getIndex()),
			this.get(Pose2dComponentsValue.Y_VALUE.getIndex()),
			angleUnit.toRotation2d(this.get(Pose2dComponentsValue.ROTATION_VALUE.getIndex()))
		);
	}

	public final Pose2d toPose2d() {
		return toPose2d(AngleUnit.RADIANS);
	}

	public final Transform2d toTransform2d(AngleUnit angleUnit) {
		Pose2d asPose = this.toPose2d(angleUnit);
		return new Transform2d(asPose.getTranslation(), asPose.getRotation());
	}

	public final Transform2d toTransform2d() {
		return toTransform2d(AngleUnit.RADIANS);
	}

	public final Translation3d toTranslation3d() {
		return new Translation3d(
			this.get(Pose3dComponentsValue.X_VALUE.getIndex()),
			this.get(Pose3dComponentsValue.Y_VALUE.getIndex()),
			this.get(Pose3dComponentsValue.Z_VALUE.getIndex())
		);
	}

	public final Twist2d toTwist2d() {
		Pose2d asPose = this.toPose2d(AngleUnit.RADIANS);
		return new Twist2d(asPose.getX(), asPose.getY(), asPose.getRotation().getRadians());
	}

	public final Rotation3d toRotation3d() {
		return new Rotation3d(this.get(0), this.get(1), this.get(2));
	}

}
