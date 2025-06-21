package frc.robot.vision.objectdetection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.ObjectData;
import frc.utils.Filter;
import frc.utils.math.AngleUnit;
import frc.utils.math.ObjectDetectionMath;
import frc.utils.pose.PoseUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class LimeLightObjectDetector implements ObjectDetector {

	private final String logPath;
	private final String cameraNetworkTablesName;
	private final Pose3d cameraPose;

	private Filter<? super ObjectData> filter;
	private Optional<ObjectData> closestObject;

	private final NetworkTableEntry closestObjectTxEntry;
	private final NetworkTableEntry closestObjectTyEntry;
	private final NetworkTableEntry closestObjectNameEntry;
	private final NetworkTableEntry closestObjectPipelineLatencyEntry;
	private final NetworkTableEntry closestObjectCaptureLatencyEntry;
	private final NetworkTableEntry t2dEntry;
	private final NetworkTableEntry allObjectsEntry;

	public LimeLightObjectDetector(String logPath, String cameraNetworkTablesName, Pose3d cameraPose) {
		this.logPath = logPath;
		this.cameraNetworkTablesName = cameraNetworkTablesName;
		this.cameraPose = cameraPose;
		setFilter(isObjectTooFarAway(VisionConstants.MAX_VALID_ALGAE_DISTANCE_METERS));

		closestObjectTxEntry = getLimelightNetworkTableEntry("tx");
		closestObjectTyEntry = getLimelightNetworkTableEntry("ty");
		closestObjectNameEntry = getLimelightNetworkTableEntry("tdclass");
		closestObjectPipelineLatencyEntry = getLimelightNetworkTableEntry("tl");
		closestObjectCaptureLatencyEntry = getLimelightNetworkTableEntry("cl");
		t2dEntry = getLimelightNetworkTableEntry("t2d");
		allObjectsEntry = getLimelightNetworkTableEntry("rawdetections");

		getLimelightNetworkTableEntry("camerapose_robotspace_set").setDoubleArray(PoseUtil.pose3DToPoseArray(cameraPose, AngleUnit.DEGREES));
	}

	private NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(cameraNetworkTablesName).getEntry(entryName);
	}

	public static Filter<double[]> squishedAlgaeFilter(double algaeHeightToWidthRatio, double heightToWidthRatioTolerance) {
		return (t2dEntryArray) -> {
			double detectedHorizontalPixels = t2dEntryArray[14];
			double detectedVerticalPixels = t2dEntryArray[15];
			double detectedHeightToWidthRatio = detectedVerticalPixels / detectedHorizontalPixels;
			return MathUtil.isNear(algaeHeightToWidthRatio, detectedHeightToWidthRatio, heightToWidthRatioTolerance);
		};
	}

	public static Filter<ObjectData> isObjectTooFarAway(double maxValidDistanceMeters) {
		;
		return (data) -> Math.abs(data.getRobotRelativeEstimatedTranslation().getX()) < maxValidDistanceMeters;
	}

	public static Optional<Translation2d> filterSquishedAlgae(
		double txEntryValue,
		double tyEntryValue,
		Filter<double[]> t2dEntrySquishedAlgaeFilter,
		double[] t2dEntryArray,
		double[] allObjectsEntryArray
	) {
		Optional<Integer> firstCellIndexInAllObjectsArray = ObjectDetectionHelpers
			.getObjectsFirstCellIndexInAllObjectsArray(txEntryValue, tyEntryValue, allObjectsEntryArray);
		if (firstCellIndexInAllObjectsArray.isEmpty()) {
			return Optional.empty();
		}

		Translation2d algaeCenterPixel = ObjectDetectionMath.getObjectCenterPixel(allObjectsEntryArray, firstCellIndexInAllObjectsArray.get());
		boolean isAlgaeSquished = !t2dEntrySquishedAlgaeFilter.apply(t2dEntryArray);
		boolean isAlgaeCutOffOnCorner = ObjectDetectionHelpers.getNumberOfObjectCornersOnPictureEdge(
			allObjectsEntryArray,
			firstCellIndexInAllObjectsArray.get(),
			(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getX(),
			(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getY(),
			VisionConstants.EDGE_PIXEL_TOLERANCE
		) >= 3;

		if (!isAlgaeSquished && !isAlgaeCutOffOnCorner) {
			return Optional.of(algaeCenterPixel);
		}
//		if (isAlgaeSquished && !isAlgaeCutOffOnCorner) {
//			return Optional.of(
//				ObjectDetectionMath.findRealSquishedAlgaeCenter(
//					algaeCenterPixel,
//					algaeHeightToWidthRatio,
//					(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getX(),
//					(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getY()
//				)
//			);
//		}
		else {
			return Optional.empty();
		}
	}

	@Override
	public Optional<ObjectData> getClosestObjectData() {
		Optional<ObjectType> objectType = ObjectDetectionHelpers.getObjectType(closestObjectNameEntry);

		if (objectType.isEmpty()
//				|| squishedAlgaeFilter(VisionConstants.ALGAE_HEIGHT_TO_WIDTH_RATIO, VisionConstants.ALGAE_HEIGHT_TO_WIDTH_RATIO_TOLERANCE)
//					.apply(t2dEntry.getDoubleArray(new double[0]))
		) {
			return Optional.empty();
		}
		return Optional.of(
			ObjectDetectionHelpers.getObjectData(
				closestObjectTxEntry,
				closestObjectTyEntry,
				closestObjectPipelineLatencyEntry,
				closestObjectCaptureLatencyEntry,
				objectType.get(),
				cameraPose
			)
		);
	}

	@Override
	public void log() {
		if (closestObject.isPresent()) {
			Logger.recordOutput(
				logPath + "closestObjectTranslation",
				new Pose2d(closestObject.get().getRobotRelativeEstimatedTranslation(), new Rotation2d())
			);
		}
	}

	@Override
	public void update() {
		closestObject = getFilteredClosestObjectData();
		log();
	}

	@Override
	public void setFilter(Filter<? super ObjectData> newFilter) {
		this.filter = newFilter;
	}

	@Override
	public Filter<? super ObjectData> getFilter() {
		return filter;
	}

}
