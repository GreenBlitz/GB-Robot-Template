package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.poseestimator.observations.Observation;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.NoSuchElementException;
import java.util.Optional;

public class PoseEstimatorMath {

    private Pose2d odometryPose;
    private Pose2d estimatedPose;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer;
    private final Matrix<N3, N1> qStdDevs;
    private final SwerveDriveKinematics kinematics;
    private SwerveDriveWheelPositions lastWheelPositions;
    private Rotation2d lastGyroAngle;
    private boolean isFirstOdometryUpdate;
    private static PoseEstimatorMath instance;

    public static PoseEstimatorMath getInstance() {
        if(instance == null) {
            instance = new PoseEstimatorMath();
        }
        return instance;
    }

    private PoseEstimatorMath() {
        odometryPose = new Pose2d();
        estimatedPose = new Pose2d();
        poseBuffer = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
        qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
        lastWheelPositions = new SwerveDriveWheelPositions(
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                });
        lastGyroAngle = new Rotation2d();
        isFirstOdometryUpdate = true;
//        kinematics = SwerveConstants.KINEMATICS;
        kinematics = null; //todo - fix when SwerveConstants are made;

        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS.get(i, 0), 2));
        }
    }

    public void addOdometryObservation(OdometryObservation observation) {
        setInitialValuesAtStart();

        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.getWheelPositions());
        lastWheelPositions = observation.getWheelPositions();

        boolean isGyroConnected = observation.getGyroAngle() != null;

        if (isGyroConnected) {
            twist = updateDeltaTheta(twist,observation);
        }
        odometryPose = odometryPose.exp(twist);
        poseBuffer.addSample(observation.getTimestamp(), odometryPose);
        estimatedPose = estimatedPose.exp(twist);
    }

    private void setInitialValuesAtStart() {
        if (isFirstOdometryUpdate) {
//            lastWheelPositions = SWERVE.getSwerveWheelPositions(0);todo - fix when swerve is made
//            lastGyroAngle = SWERVE.getOdometryYawUpdates()[0];
            isFirstOdometryUpdate = false;
        }
    }

    private Twist2d updateDeltaTheta(Twist2d twist, OdometryObservation observation) {
        lastGyroAngle = observation.getGyroAngle();
        return new Twist2d(twist.dx, twist.dy, observation.getGyroAngle().minus(lastGyroAngle).getRadians());
    }

    private Transform2d useKalmanOnTransform(VisionObservation observation, Pose2d estimateAtTime) {
        double[] squaredVisionMatrix = getSquaredVisionMatrix(observation);
        Matrix<N3, N3> visionK = kalmanFilterAlgorithm(squaredVisionMatrix);
        Transform2d diffFromOdometry = new Transform2d(estimateAtTime, observation.getVisionPose());
        return scaleDiffFromKalman(diffFromOdometry, visionK);
    }

    public void addVisionObservation(VisionObservation observation) {
        if(isObservationTooOld(observation)) {
            return;
        }
        Optional<Pose2d> sample = poseBuffer.getSample(observation.getTimestamp());
        if (sample.isEmpty()) {
            return;
        }
        Transform2d sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
        estimatedPose = estimateAtTime
                .plus(useKalmanOnTransform(observation,estimateAtTime))
                .plus(sampleToOdometryTransform);
    }

    private boolean isObservationTooOld(Observation observation) {
        try {
            return poseBuffer.getInternalBuffer().lastKey() - PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS > observation.getTimestamp();
        }
        catch (NoSuchElementException ex) {
            return true;
        }
    }

    private double[] getSquaredVisionMatrix(VisionObservation observation) {
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = observation.getStdDevs().get(i, 0) * observation.getStdDevs().get(i, 0);
        }
        return r;
    }

    private Matrix<N3, N3> kalmanFilterAlgorithm(double[] squaredVisionMatrix) {
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            }
            else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * squaredVisionMatrix[row])));
            }
        }
        return visionK;
    }

    private Transform2d scaleDiffFromKalman(Transform2d diffFromOdometry, Matrix<N3, N3> visionK) {
        Matrix<N3, N1> kTimesDiff = visionK.times(
                VecBuilder.fill(diffFromOdometry.getX(), diffFromOdometry.getY(), diffFromOdometry.getRotation().getRadians())
        );
        return new Transform2d(
                kTimesDiff.get(0, 0),
                kTimesDiff.get(1, 0),
                Rotation2d.fromRadians(kTimesDiff.get(2, 0))
        );
    }

    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        lastGyroAngle = initialPose.getRotation();
        odometryPose = initialPose;
        poseBuffer.clear();
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public void setOdometryStandardDeviations(double x, double y, double rotation) {
        Vector<N3> newQStdDevs = VecBuilder.fill(x, y, rotation);
        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(newQStdDevs.get(i, 0), 2));
        }
    }

}
