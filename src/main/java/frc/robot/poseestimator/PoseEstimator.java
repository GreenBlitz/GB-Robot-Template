package frc.robot.poseestimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.poseestimator.observations.VisionObservation;

public class PoseEstimator implements IPoseEstimator{
    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveModulePosition[] wheelPositions;
    private SwerveDriveKinematics kinematics;
    private Rotation2d gyroAngle;
    private Pose2d startPose;
    private  StringPublisher posePublisher;
    public PoseEstimator(SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] wheelPositions,
            Pose2d startPose,StringTopic poseTopic){
        this.gyroAngle = gyroAngle;
        this.kinematics = kinematics;
        this.wheelPositions = wheelPositions;
        this.startPose = startPose;
        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.kinematics,
                this.gyroAngle,
                this.wheelPositions,
                this.startPose
        );
        this.posePublisher = poseTopic.publish();
    }

    @Override
    public void updateOdometry(OdometryObservation odometryObservation) {
        poseEstimator.update(odometryObservation.getGyroAngle(),odometryObservation.getWheelPositions());
    }

    @Override
    public void resetOdometry(SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, Pose2d pose) {

    }

    @Override
    public Pose2d getOdometryPose() {
        return null;
    }

    @Override
    public void updateVision(VisionObservation visionObservation) {
        poseEstimator.addVisionMeasurement(visionObservation.getVisionPose(),visionObservation.getTimestamp());
    }

    @Override
    public Pose2d getVisionPose() {
        return null;
    }

    @Override
    public void setStandardDeviations(Matrix<N3, N1> standardDeviations) {
        poseEstimator.setVisionMeasurementStdDevs(standardDeviations);
    }

    @Override
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(gyroAngle,wheelPositions,newPose);
    }

//    public void periodic(){
//        posePublisher.set(JsonConverter.readFromObject(getEstimatedPose()));
//    }

}
