package frc.robot.vision.objectdetection;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.ObjectData;
import frc.utils.math.ObjectDetectionMath;
import frc.utils.time.TimeUtil;

import java.util.Optional;

public class ObjectDetectionHelpers {

	public static Pair<Double, Double> txNoCrossAndTyNoCrossToTxAndTy(double txNoCross, double tyNoCross) {
		double tx = txNoCross - (VisionConstants.LIMELIGHT_3_HORIZONTAL_FOV.getDegrees() / 2);
		double ty = tyNoCross - (VisionConstants.LIMELIGHT_3_VERTICAL_FOV.getDegrees() / 2);
		return new Pair<>(tx, ty);
	}

	public static Optional<Integer> getObjectsFirstCellIndexInAllObjectsArray(
		double txEntryValue,
		double tyEntryValue,
		double[] allObjectsEntryArray
	) {
		int objectAmount = allObjectsEntryArray.length / VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY;

		for (int i = 0; i < objectAmount; i++) {
			int firstCell = VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY * i;
			double allObjectsEntryTxNoCrossValue = allObjectsEntryArray[firstCell + AllObjectsEntryIndexes.TX_NO_CROSS.getIndex()];
			double allObjectsEntryTyNoCrossValue = allObjectsEntryArray[firstCell + AllObjectsEntryIndexes.TY_NO_CROSS.getIndex()];

			Pair<Double, Double> allObjectsEntryTxAndTyValues = txNoCrossAndTyNoCrossToTxAndTy(
				allObjectsEntryTxNoCrossValue,
				allObjectsEntryTyNoCrossValue
			);

			if (allObjectsEntryTxAndTyValues.getFirst() == txEntryValue && allObjectsEntryTxAndTyValues.getSecond() == tyEntryValue) {
				return Optional.of(firstCell);
			}
		}
		return Optional.empty();
	}

	public static int getNumberOfObjectCornersOnPictureEdge(
		double[] allObjectsEntryArray,
		int objectFirstCellIndex,
		int pictureWidthPixels,
		int pictureHeightPixels,
		int edgePixelTolerance
	) {
		int numberOfCornersOnPictureEdge = 0;
		Translation2d[] objectFrameCorners = getAllObjectFrameCorners(allObjectsEntryArray, objectFirstCellIndex);

		for (Translation2d corner : objectFrameCorners) {
			if (ObjectDetectionMath.isPixelOnEdgeOfPicture(corner, pictureWidthPixels, pictureHeightPixels, edgePixelTolerance)) {
				numberOfCornersOnPictureEdge++;
			}
		}
		return numberOfCornersOnPictureEdge;
	}

	public static Translation2d[] getAllObjectFrameCorners(double[] allObjectsEntryArray, int objectFirstCellIndex) {
		return new Translation2d[] {
			new Translation2d(
				allObjectsEntryArray[objectFirstCellIndex + AllObjectsEntryIndexes.CORNER_0_X.getIndex()],
				allObjectsEntryArray[objectFirstCellIndex + AllObjectsEntryIndexes.CORNER_0_Y.getIndex()]
			),
			new Translation2d(
				allObjectsEntryArray[objectFirstCellIndex + AllObjectsEntryIndexes.CORNER_1_X.getIndex()],
				allObjectsEntryArray[objectFirstCellIndex + AllObjectsEntryIndexes.CORNER_1_Y.getIndex()]
			),
			new Translation2d(
				allObjectsEntryArray[objectFirstCellIndex + AllObjectsEntryIndexes.CORNER_2_X.getIndex()],
				allObjectsEntryArray[objectFirstCellIndex + AllObjectsEntryIndexes.CORNER_2_Y.getIndex()]
			),
			new Translation2d(
				allObjectsEntryArray[objectFirstCellIndex + AllObjectsEntryIndexes.CORNER_3_X.getIndex()],
				allObjectsEntryArray[objectFirstCellIndex + AllObjectsEntryIndexes.CORNER_3_Y.getIndex()]
			)};
	}

	public static Optional<ObjectType> getObjectType(NetworkTableEntry objectNameEntry) {
		String nameEntryValue = objectNameEntry.getString(VisionConstants.NAME_ENTRY_NO_OBJECT_VALUE);
		for (ObjectType type : ObjectType.values()) {
			if (type.getNameEntryValue().equals(nameEntryValue)) {
				return Optional.of(type);
			}
		}
		return Optional.empty();
	}

	public static ObjectData getObjectData(
		NetworkTableEntry txEntry,
		NetworkTableEntry tyEntry,
		NetworkTableEntry pipelineLatencyEntry,
		NetworkTableEntry captureLatencyEntry,
		ObjectType objectType,
		Pose3d cameraPose
	) {
		double centerOfObjectHeightMeters = objectType.getObjectHeightMeters() / 2;

		Rotation2d cameraRelativeObjectYaw = Rotation2d.fromDegrees(txEntry.getDouble(0));
		Rotation2d cameraRelativeObjectPitch = Rotation2d.fromDegrees(tyEntry.getDouble(0));
		Translation2d robotRelativeObjectTranslation = ObjectDetectionMath
			.getRobotRelativeTranslation(cameraRelativeObjectYaw, cameraRelativeObjectPitch, cameraPose, centerOfObjectHeightMeters);

		double totalLatency = pipelineLatencyEntry.getDouble(0) + captureLatencyEntry.getDouble(0);
		double timeStamp = TimeUtil.getCurrentTimeSeconds() - totalLatency;

		return new ObjectData(robotRelativeObjectTranslation, objectType, timeStamp);
	}

}
