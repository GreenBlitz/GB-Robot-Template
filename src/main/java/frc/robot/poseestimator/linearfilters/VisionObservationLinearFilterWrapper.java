package frc.robot.poseestimator.linearfilters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.poseestimator.observations.VisionObservation;
import org.littletonrobotics.junction.Logger;

import java.util.*;

public class VisionObservationLinearFilterWrapper {

	private final GBLinearFilter xFilter;
	private final GBLinearFilter yFilter;
	private final GBLinearFilter angleFilter;
	private final Stack<VisionObservation> internalData;
	private final String logPath;
	private Pose2d LastObservation;

	/**
	 * A wrapper for {@code ILinearFilter} using odometry and vision
	 *
	 * @param logPath:    like, the log path
	 * @param filterType: the type of the linear filter that would be applied
	 * @param modifier:   modify the behavior of the filter. In case of FIR filters, this would be the sample count (casted to an integer), and
	 *                    for IIR filters the time constant (the period is always the RobotRIO cycle time).
	 */
	public VisionObservationLinearFilterWrapper(String logPath, LinearFilterType filterType, double modifier) {
		this.logPath = logPath;
		this.xFilter = LinearFilterFactory.create(filterType, logPath, modifier);
		this.yFilter = LinearFilterFactory.create(filterType, logPath, modifier);
		this.angleFilter = LinearFilterFactory.create(filterType, logPath, modifier);
		this.internalData = new Stack<>();
		this.LastObservation = new Pose2d();
	}

	/**
	 *
	 * adds new data to the filter, transformed by the new odometry data from the within a time period. That way, the old vision observation
	 * would be updated to its new approximated position
	 *
	 * @param visionObservation:             the observation that needs to be fixed and added
	 * @param newOdometryStartingTimestamps: the starting of the new odometry data
	 * @param newOdometrySEndingTimestamps:  the ending of the new odometry data
	 * @param odometryObservationsOverTime:  interpolated buffer contains the data from the starting to the ending of the new odometry data
	 */
	private Optional<Pose2d> getFixedData(
		Pose2d visionObservation,
		double newOdometryStartingTimestamps,
		double newOdometrySEndingTimestamps,
		TimeInterpolatableBuffer<Pose2d> odometryObservationsOverTime
	) {
		Optional<Pose2d> odometryStartingPose = odometryObservationsOverTime.getSample(newOdometryStartingTimestamps);
		Optional<Pose2d> odometryEndingPose = odometryObservationsOverTime.getSample(newOdometrySEndingTimestamps);

		if (odometryStartingPose.isPresent() && odometryEndingPose.isPresent()) {
			return Optional.of(fixVisionPose(visionObservation, odometryStartingPose.get(), odometryEndingPose.get()));
		}
		return Optional.empty();
	}

	public void addData(VisionObservation data) {
		LastObservation = data.robotPose();
		internalData.add(data);
		checkSize();
		Logger.recordOutput(logPath  + "gotData", data.robotPose());
	}

	private Pose2d fixVisionPose(Pose2d visionPose, Pose2d odometryStartingPose, Pose2d odometryEndingPose) {
		Transform2d changeInPosition = odometryStartingPose.minus(odometryEndingPose);
		return visionPose.plus(changeInPosition);
	}

	public Pose2d calculateNonFixedData() {
		return new Pose2d(
			xFilter.calculateNewData(internalData.getLast().robotPose().getX()),
			yFilter.calculateNewData(internalData.getLast().robotPose().getY()),
			Rotation2d.fromRotations(internalData.getLast().robotPose().getRotation().getRotations())
		);
	}

	public Pose2d calculateFixedData(TimeInterpolatableBuffer<Pose2d> odometryObservationsOverTime) {
		// ! bad code. pls don't merge this, or fix this. Several side effects.
		double timestamp = Timer.getFPGATimestamp();
		xFilter.getFilter().reset();
		yFilter.getFilter().reset();
		angleFilter.getFilter().reset();
		for (VisionObservation data : internalData) {
			Optional<Pose2d> fixed = getFixedData(data.robotPose(), data.timestamp(), timestamp, odometryObservationsOverTime);
			fixed.ifPresent((Pose2d fixedData) -> {
				Pose2d output = new Pose2d(
						xFilter.calculateNewData(fixedData.getX()),
						yFilter.calculateNewData(fixedData.getY()),
						Rotation2d.fromRotations(angleFilter.calculateNewData(fixedData.getRotation().getRotations()))
				);
				System.out.println(Arrays.toString(new double[]{output.getX(), output.getY(), output.getRotation().getDegrees()}));
				Logger.recordOutput(logPath + "outputedFixed", output);
//				return output;
			});
		}
		System.out.println("interpolation failed");
		Logger.recordOutput("interpolation failed", timestamp);
		return new Pose2d();
	}

	protected void checkSize() {
		if (internalData.size() >= LinearFiltersConstants.MAX_SIZE) {
			internalData.remove(internalData.size() - 1);
		}
	}

}
