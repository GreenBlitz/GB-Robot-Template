package frc.robot.poseestimator.observations;

import frc.utils.Filter;

import java.util.Optional;

public class OdometryFilterer {

    private final Filter<OdometryObservation> filter;

    public OdometryFilterer(Filter<OdometryObservation> filter) {
        this.filter = filter;
    }

    public Optional<OdometryObservation> getFilteredObservation(OdometryObservation observation) {
        if (!filter.apply(observation)) {
            observation = null;
        }
        return Optional.ofNullable(observation);
    }
}
