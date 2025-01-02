package frc.robot.poseestimator.helpers;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;
import frc.robot.poseestimator.Pose3dComponentsValue;

public record StandardDeviations3D(
	double xAxisStandardDeviations,
	double yAxisStandardDeviations,
	double zAxisStandardDeviations,
	double rollStandardDeviations,
	double pitchStandardDeviations,
	double yawStandardDeviations
) {

	public StandardDeviations3D(double[] standardDeviations) {
		this(
			standardDeviations[Pose3dComponentsValue.X_VALUE.getIndex()],
			standardDeviations[Pose3dComponentsValue.Y_VALUE.getIndex()],
			standardDeviations[Pose3dComponentsValue.Z_VALUE.getIndex()],
			standardDeviations[Pose3dComponentsValue.ROLL_VALUE.getIndex()],
			standardDeviations[Pose3dComponentsValue.PITCH_VALUE.getIndex()],
			standardDeviations[Pose3dComponentsValue.YAW_VALUE.getIndex()]
		);
	}

	public StandardDeviations3D() {
		this(0, 0, 0, 0, 0, 0);
	}

	public Matrix<N6, N1> getStandardDeviations3D() {
		return VecBuilder.fill(
			xAxisStandardDeviations,
			yAxisStandardDeviations,
			zAxisStandardDeviations,
			rollStandardDeviations,
			pitchStandardDeviations,
			yawStandardDeviations
		);
	}

}
