package frc.utils.gbmeth;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
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

}
