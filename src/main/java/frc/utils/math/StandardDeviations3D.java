package frc.utils.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import frc.robot.poseestimator.Pose3dComponentsValue;
import frc.utils.alerts.Alert;

public class StandardDeviations3D {

	private double xAxisStandardDeviations;
	private double yAxisStandardDeviations;
	private double zAxisStandardDeviations;
	private double rollStandardDeviations;
	private double pitchStandardDeviations;
	private double yawStandardDeviations;

	public StandardDeviations3D(double[] standardDeviations) {
		if (standardDeviations.length != Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT) {
			new Alert(
				Alert.AlertType.ERROR,
				"When constructing StandardDeviations record: array length expected to be "
					+ Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT
					+ " but is "
					+ standardDeviations.length
			).report();
		}

		this.xAxisStandardDeviations = standardDeviations[Pose3dComponentsValue.X_VALUE.getIndex()];
		this.yAxisStandardDeviations = standardDeviations[Pose3dComponentsValue.Y_VALUE.getIndex()];
		this.zAxisStandardDeviations = standardDeviations[Pose3dComponentsValue.Z_VALUE.getIndex()];
		this.rollStandardDeviations = standardDeviations[Pose3dComponentsValue.ROLL_VALUE.getIndex()];
		this.pitchStandardDeviations = standardDeviations[Pose3dComponentsValue.PITCH_VALUE.getIndex()];
		this.yawStandardDeviations = standardDeviations[Pose3dComponentsValue.YAW_VALUE.getIndex()];
	}

	public StandardDeviations3D() {}

	public Matrix<N6, N1> getAsColumnVector() {
		return VecBuilder.fill(
			xAxisStandardDeviations,
			yAxisStandardDeviations,
			zAxisStandardDeviations,
			rollStandardDeviations,
			pitchStandardDeviations,
			yawStandardDeviations
		);
	}

	public double getXAxisStandardDeviations() {
		return xAxisStandardDeviations;
	}

	public double getYAxisStandardDeviations() {
		return yAxisStandardDeviations;
	}

	public double getZAxisStandardDeviations() {
		return zAxisStandardDeviations;
	}

	public double getRollStandardDeviations() {
		return rollStandardDeviations;
	}

	public double getPitchStandardDeviations() {
		return pitchStandardDeviations;
	}

	public double getYawStandardDeviations() {
		return yawStandardDeviations;
	}

}
