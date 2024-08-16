package frc.robot.poseestimator.observations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionObservation extends Observation{

    private Pose2d visionPose;

    private Matrix<N3, N1> standardDeviations;

    public VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> standardDeviations) {
        this.visionPose = visionPose;
        this.timestamp = timestamp;
        this.standardDeviations = standardDeviations;
    }

    public Pose2d getVisionPose() {
        return visionPose;
    }

    public void setVisionPose(Pose2d visionPose) {
        this.visionPose = visionPose;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(double timestamp) {
        this.timestamp = timestamp;
    }

    public Matrix<N3, N1> getStandardDeviations() {
        return standardDeviations;
    }

    public void setStandardDeviations(Matrix<N3, N1> standardDeviations) {
        this.standardDeviations = standardDeviations;
    }

}
