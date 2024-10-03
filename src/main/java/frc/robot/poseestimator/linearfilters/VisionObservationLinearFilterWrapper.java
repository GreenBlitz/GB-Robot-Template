package frc.robot.poseestimator.linearfilters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.*;

public class VisionObservationLinearFilterWrapper {

	private final ILinearFilter<Pose2d> linearFilter;
	private final Stack<Pose2d> updatedObservations;

	public VisionObservationLinearFilterWrapper(LinearFilterType filterType) {
		linearFilter = switch (filterType) {
			case highPassIIR -> new HighPassIIR();
            case singlePoleIIR -> new SigSinglePoseIIR();
			case movingAverageFIR -> new MovingAverageFIR();
		};
		this.updatedObservations = new Stack<>();
	}

	/**
	 *
	 * adds new data to the filter, exponented by the new odometry data from the within a time period. That way, the old vision
	 * observation would be updated to its new approximated position
	 *
	 * @param visionObservation:             the observation that needs to be fixed and added
	 * @param newOdometryStartingTimestamps: the starting of the new odometry data
	 * @param newOdometrySEndingTimestamps:  the ending of the new odometry data
	 * @param odometryObservationsOverTime:  navigable map contains the data from the starting to the ending of the new odometry
	 *                                       data
	 */
	public void addFixedData(
		VisionObservation visionObservation,
		double newOdometryStartingTimestamps,
		double newOdometrySEndingTimestamps,
		NavigableMap<Double, Pose2d> odometryObservationsOverTime
	) {
		try {
			updatedObservations.add(
					fixVisionPose(
							visionObservation.visionPose(),
							(Pose2d[]) odometryObservationsOverTime.subMap(newOdometryStartingTimestamps, newOdometrySEndingTimestamps)
									.values()
									.toArray()
					)
			);
		} catch (ClassCastException e) {
			return;
		}
	}

	public void addRawData(VisionObservation data) {
		updatedObservations.add(data.visionPose());
	}

	private Pose2d fixVisionPose(Pose2d visionPose, Pose2d[] newOdometryData) {
		Pose2d output = visionPose;
		for (int i = 0; i <= newOdometryData.length - 1; i++) {
			Pose2d odometryPose = newOdometryData[i];
			Pose2d nextOdometryPose = newOdometryData[i + 1];

			Transform2d poseDifferenceFromSample = new Transform2d(odometryPose, nextOdometryPose);
			Transform2d sampleDifferenceFromPose = poseDifferenceFromSample.inverse();
			output = output.plus(sampleDifferenceFromPose);
		}

		return output;
	}

	public Pose2d calculateFilteredPose() {
		return linearFilter.calculateNewData(updatedObservations.getLast());
	}

}
