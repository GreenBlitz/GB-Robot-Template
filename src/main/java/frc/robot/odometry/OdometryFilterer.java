package frc.robot.odometry;

import frc.robot.poseestimator.observations.OdometryObservation;
import frc.utils.Filter;

import java.util.Optional;

public class OdometryFilterer {

    private final Filter<OdometryObservation> filter;

    public OdometryFilterer(Filter<OdometryObservation> filter) {
        this.filter = filter;
    }

    public Optional<OdometryObservation> getFilteredObservation(OdometryObservation observation) {
        return !filter.apply(observation) ? Optional.empty() : Optional.of(observation);
    }

}
