package frc.utils.gbmeth;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N6;
import frc.robot.poseestimator.Pose3dComponentsValue;

import java.util.Collection;

public class GBVector6D extends GBVector<N6> {

	public GBVector6D(double[] data) {
		super(data);
	}

	public <E extends Num> GBVector6D(Vector<E> vector) {
		super(vector);
	}

	public GBVector6D(Collection<Double> data) {
		super(data);
	}

	public GBVector3D to2DPose() {
		return new GBVector3D(
			new double[] {
				this.get(Pose3dComponentsValue.X_VALUE.getIndex()),
				this.get(Pose3dComponentsValue.Y_VALUE.getIndex()),
				this.get(Pose3dComponentsValue.YAW_VALUE.getIndex())}
		);
	}

	public GBVector3D to3DTranslation() {
		return new GBVector3D(
			new double[] {
				this.get(Pose3dComponentsValue.X_VALUE.getIndex()),
				this.get(Pose3dComponentsValue.Y_VALUE.getIndex()),
				this.get(Pose3dComponentsValue.Z_VALUE.getIndex())}
		);
	}

}
