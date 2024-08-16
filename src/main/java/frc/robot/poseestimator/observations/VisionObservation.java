package frc.robot.poseestimator.observations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionObservation extends Observation{

    private Pose2d visionPose;

    private Matrix<N3, N1> stdDevs;

    public VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        this.visionPose = visionPose;
        this.timestamp = timestamp;
        this.stdDevs = stdDevs;
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

    public Matrix<N3, N1> getStdDevs() {
        return stdDevs;
    }

    public void setStdDevs(Matrix<N3, N1> stdDevs) {
        this.stdDevs = stdDevs;
    }

}
