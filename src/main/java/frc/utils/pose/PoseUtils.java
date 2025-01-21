package frc.utils.pose;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.poseestimator.Pose2dComponentsValue;
import frc.robot.poseestimator.Pose3dComponentsValue;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.vision.data.VisionData;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public class PoseUtils {

	public static Pose3d poseArrayToPose3D(double[] poseArray, AngleUnit angleUnit) {
		int requiredAmount = Pose3dComponentsValue.POSE3D_COMPONENTS_AMOUNT;
		if (poseArray.length != requiredAmount) {
			new Alert(Alert.AlertType.WARNING, "gotBadPoseArrayWith" + poseArray.length + "ElementsInsteadOf" + requiredAmount);
		}
		return new Pose3d(
			new Translation3d(
				poseArray[Pose3dComponentsValue.X_VALUE.getIndex()],
				poseArray[Pose3dComponentsValue.Y_VALUE.getIndex()],
				poseArray[Pose3dComponentsValue.Z_VALUE.getIndex()]
			),
			new Rotation3d(
				angleUnit.toRotation2d(poseArray[Pose3dComponentsValue.ROLL_VALUE.getIndex()]).getRadians(),
				angleUnit.toRotation2d(poseArray[Pose3dComponentsValue.PITCH_VALUE.getIndex()]).getRadians(),
				angleUnit.toRotation2d(poseArray[Pose3dComponentsValue.YAW_VALUE.getIndex()]).getRadians()
			)
		);
	}

	public static Pose2d poseArrayToPose2D(double[] poseArray, AngleUnit angleUnit) {
		int requiredAmount = Pose2dComponentsValue.POSE2D_COMPONENTS_AMOUNT;
		if (poseArray.length != requiredAmount) {
			new Alert(Alert.AlertType.WARNING, "gotBadPoseArrayWith" + poseArray.length + "ElementsInsteadOf" + requiredAmount);
		}
		return new Pose2d(
			poseArray[Pose2dComponentsValue.X_VALUE.getIndex()],
			poseArray[Pose2dComponentsValue.X_VALUE.getIndex()],
			angleUnit.toRotation2d(poseArray[Pose2dComponentsValue.ROTATION_VALUE.getIndex()])
		);
	}

	public static TimedValue<Rotation2d> visionDataToHeadingData(VisionData visionData) {
		return new TimedValue<>(visionData.getEstimatedPose().getRotation().toRotation2d(), visionData.getTimestamp());
	}

	public static Function<Pose2d, Vector<N3>> poseToVector = pose -> new Vector<>(
		new SimpleMatrix(new double[][] {{pose.getX(), pose.getY(), pose.getRotation().getRadians()}})
	);


	private static Vector<N3> applyFunctionOnPoseElements(List<Vector<N3>> dataset, Function<List<Double>, Double> function) {
		List<Double> XSet = new ArrayList<>();
		List<Double> YSet = new ArrayList<>();
		List<Double> AngleSet = new ArrayList<>();
		for (Vector<N3> data : dataset) {
			XSet.add(data.get(0));
			YSet.add(data.get(1));
			AngleSet.add(data.get(2));
		}
		return poseToVector.apply(new Pose2d(function.apply(XSet), function.apply(YSet), Rotation2d.fromRadians(function.apply(AngleSet))));
	}

	public static Vector<N3> meanOfPose(List<Vector<N3>> dataset) {
		return applyFunctionOnPoseElements(dataset, PoseUtils::mean);
	}

	public static Vector<N3> stdDevVector(List<Vector<N3>> dataset) {
		return applyFunctionOnPoseElements(dataset, PoseUtils::calculateStandardDeviation);
	}

	public static double mean(List<Double> dataset) {
		double sum = 0;
		for (double data : dataset) {
			sum += data;
		}
		return sum / dataset.size();
	}

	public static double calculateStandardDeviation(List<Double> dataset) {
		double mean = mean(dataset);
		double squaredDeviation = 0;
		for (double data : dataset) {
			squaredDeviation += Math.pow(data - mean, 2);
		}
		return Math.sqrt(squaredDeviation / dataset.size());
	}

}
