package frc.robot.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.ObjectData;
import frc.utils.math.ObjectDetectionMath;
import frc.utils.time.TimeUtil;

import java.util.Optional;

public class NetworkTableEntriesHelpers {

	public static Optional<Integer> getObjectsFirstCellInAllObjectsArray(
		double txEntryValue,
		double tyEntryValue,
		double[] allObjectsEntryArray
	) {
		int objectAmount = allObjectsEntryArray.length / VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY;

		for (int i = 0; i < objectAmount; i++) {
			int firstCell = VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY * i;
			double allObjectsEntryTxValue = allObjectsEntryArray[firstCell + 1];
			double allObjectsEntryTyValue = allObjectsEntryArray[firstCell + 2];

			if (allObjectsEntryTxValue == txEntryValue && allObjectsEntryTyValue == tyEntryValue) {
				return Optional.of(firstCell);
			}
		}
		return Optional.empty();
	}

	public static int getNumberOfObjectCornersOnPictureEdge(
		double[] allObjectsEntryArray,
		int allObjectsEntryArrayFirstCell,
		double pictureMaxXValue,
		double pictureMaxYValue,
		double edgePixelTolerance
	) {
		int numberOfCornersOnPictureEdge = 0;

		// rename to make lines shorter :( pls dont kill me
		int firstCell = allObjectsEntryArrayFirstCell;

		Translation2d[] objectFrameCorners = {
			new Translation2d(allObjectsEntryArray[firstCell + 4], allObjectsEntryArray[firstCell + 5]),
			new Translation2d(allObjectsEntryArray[firstCell + 6], allObjectsEntryArray[firstCell + 7]),
			new Translation2d(allObjectsEntryArray[firstCell + 8], allObjectsEntryArray[firstCell + 9]),
			new Translation2d(allObjectsEntryArray[firstCell + 10], allObjectsEntryArray[firstCell + 11])};

		for (Translation2d corner : objectFrameCorners) {
			if (ObjectDetectionMath.isPixelOnEdgeOfPicture(corner, pictureMaxXValue, pictureMaxYValue, edgePixelTolerance)) {
				numberOfCornersOnPictureEdge++;
			}
		}
		return numberOfCornersOnPictureEdge;
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

	public static ObjectData getClosestObjectData(
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
