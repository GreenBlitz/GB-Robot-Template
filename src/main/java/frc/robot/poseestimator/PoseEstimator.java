package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;
import java.util.NoSuchElementException;
import java.util.Optional;

public class PoseEstimator implements IPoseEstimator {

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer;
    private final SwerveDriveKinematics kinematics;
    private Pose2d odometryPose;
    private Pose2d estimatedPose;
    private Matrix<N3, N1> standardDeviations;
    private SwerveDriveWheelPositions lastWheelPositions;
    private Rotation2d lastGyroAngle;
    private VisionObservation lastVisionObservation;

    public PoseEstimator (
            SwerveDriveKinematics kinematics,
            SwerveDriveWheelPositions initialWheelPositions,
            Rotation2d initialGyroAngle
    ) {
        this.odometryPose = new Pose2d();
        this.estimatedPose = new Pose2d();
        this.poseBuffer = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
        this.kinematics = kinematics;
        this.lastWheelPositions = initialWheelPositions;
        this.lastGyroAngle = initialGyroAngle;
        this.standardDeviations = new Matrix<>(Nat.N3(), Nat.N1());
        setOdometryStandardDeviations(PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS);
    }

    private void addOdometryObservation(OdometryObservation observation) {
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        this.lastGyroAngle = observation.gyroAngle();
        twist = PoseEstimatorMath.addGyroToTwistCalculations(observation, twist, lastGyroAngle);
        this.lastWheelPositions = observation.wheelPositions();
        this.odometryPose = odometryPose.exp(twist);
        this.estimatedPose = estimatedPose.exp(twist);
        poseBuffer.addSample(observation.timestamp(), odometryPose);
    }

    private void addVisionObservation(VisionObservation observation) {
        Optional<Pose2d> sample = poseBuffer.getSample(observation.timestamp());
        if (!sample.isEmpty()) {
            estimatedPose = PoseEstimatorMath.combineVisionObservationAndOdometrySample(
                    sample,
                    observation,
                    estimatedPose,
                    odometryPose,
                    standardDeviations
            );
        }
    }

    private boolean isObservationTooOld(VisionObservation visionObservation) {
        try {
            return poseBuffer.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS > visionObservation.timestamp();
        }
        catch (NoSuchElementException ignored) {
            return true;
        }
    }

    @Override
    public void resetPose(Pose2d initialPose) {
        this.estimatedPose = initialPose;
        this.lastGyroAngle = initialPose.getRotation();
        this.odometryPose = initialPose;
        poseBuffer.clear();
    }

    @Override
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public void setOdometryStandardDeviations(Vector<N3> newStandardDeviations) {
        for (int row = 0; row < newStandardDeviations.getNumRows(); row++) {
            standardDeviations.set(row, 0, Math.pow(newStandardDeviations.get(row, 0), PoseEstimatorConstants.KALMAN_EXPONENT));
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
        this.lastVisionObservation = visionObservation;
        if(!isObservationTooOld(visionObservation)) {
            addVisionObservation(visionObservation);
        }
    }

    @Override
    public Optional<Pose2d> getVisionPose() {
        return Optional.of(lastVisionObservation.visionPose());
    }

    @Override
    public void setStandardDeviations(Matrix<N3, N1> standardDeviations) {
        this.standardDeviations = standardDeviations;
    }

}
