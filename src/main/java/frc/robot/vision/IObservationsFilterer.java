package frc.robot.vision;

import frc.robot.poseestimator.observations.VisionObservation;

import java.util.List;

public interface IObservationsFilterer {

	public abstract List<VisionObservation> getAllFilteredData();

}
