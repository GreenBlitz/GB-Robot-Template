package frc.robot.subsystems;

import frc.robot.poseestimator.VisionObservation;

public interface Vision {
    void updateVision(VisionObservation visionObservation);

    void add()
}
