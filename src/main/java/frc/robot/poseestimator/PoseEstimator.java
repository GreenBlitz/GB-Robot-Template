package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.observations.Observation;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import java.util.NoSuchElementException;
import java.util.Optional;

public class PoseEstimator implements IPoseEstimator {

    private Pose2d odometryPose;
    private Pose2d estimatedPose;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer;
    private Matrix<N3, N1> standardDeviations;
    private final SwerveDriveKinematics kinematics;
    private SwerveDriveWheelPositions lastWheelPositions;
    private Rotation2d lastGyroAngle;
    private VisionObservation lastVisionObservation;

    public PoseEstimator (
            SwerveDriveKinematics kinematics,
            SwerveDriveWheelPositions initialWheelPositions,
            Rotation2d initialGyroAngle
    ) {
        odometryPose = new Pose2d();
        estimatedPose = new Pose2d();
        poseBuffer = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
        standardDeviations = new Matrix<>(Nat.N3(), Nat.N1());
        this.kinematics = kinematics;
        this.lastWheelPositions = initialWheelPositions;
        this.lastGyroAngle = initialGyroAngle;
        for (int i = 0; i < 3; ++i) {
            standardDeviations.set(
                    i,
                    0,
                    Math.pow(PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS.get(i, 0),
                            PoseEstimatorConstants.KALMAN_EXPONENT
                    )
            );
        }
    }

    public void addOdometryObservation(OdometryObservation observation) {
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.getWheelPositions());
        lastWheelPositions = observation.getWheelPositions();
        lastGyroAngle = observation.getGyroAngle();
        twist = PoseEstimatorMath.addGyroToTwistCalculations(observation, twist, lastGyroAngle);
        odometryPose = odometryPose.exp(twist);
        poseBuffer.addSample(observation.getTimestamp(), odometryPose);
        estimatedPose = estimatedPose.exp(twist);
    }

    public void addVisionObservation(VisionObservation observation) {
        lastVisionObservation = observation;
        if(isObservationTooOld(observation)) {
            return;
        }
        Optional<Pose2d> sample = poseBuffer.getSample(observation.getTimestamp());
        if (sample.isEmpty()) {
            return;
        }
        estimatedPose = PoseEstimatorMath.combineVisionObservationAndOdometrySample(
                sample,
                observation,
                estimatedPose,
                odometryPose,
                standardDeviations
        );
    }

    private boolean isObservationTooOld(Observation observation) {
        try {
            return poseBuffer.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS > observation.getTimestamp();
        }
        catch (NoSuchElementException ex) {
            return true;
        }
    }

    @Override
    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        lastGyroAngle = initialPose.getRotation();
        odometryPose = initialPose;
        poseBuffer.clear();
    }

    @Override
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public void setOdometryStandardDeviations(double x, double y, double rotation) {
        Vector<N3> newStandardDeviations = VecBuilder.fill(x, y, rotation);
        for (int i = 0; i < newStandardDeviations.getNumRows(); ++i) {
            standardDeviations.set(i, 0, Math.pow(newStandardDeviations.get(i, 0), PoseEstimatorConstants.KALMAN_EXPONENT));
        }
    }

    @Override
    public void updateOdometry(OdometryObservation odometryObservation) {
        addOdometryObservation(odometryObservation);
    }

    @Override
    public void resetOdometry(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, Pose2d pose) {
        this.lastWheelPositions = wheelPositions;
        this.lastGyroAngle = gyroAngle;
        this.odometryPose = pose;
        poseBuffer.clear();
    }

    @Override
    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    @Override
    public void updateVision(VisionObservation visionObservation) {
        addVisionObservation(visionObservation);
    }

    @Override
    public Pose2d getVisionPose() {
        if(lastVisionObservation != null) {
            return lastVisionObservation.getVisionPose();
        }
        return null;
    }

    @Override
    public void setStandardDeviations(Matrix<N3, N1> standardDeviations) {
        this.standardDeviations = standardDeviations;
    }

}
