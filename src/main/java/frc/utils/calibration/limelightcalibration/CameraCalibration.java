package frc.utils.calibration.limelightcalibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.utils.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.Supplier;

public class CameraCalibration extends Command {

    private final Limelight limelight;
    private final Pose3d tagToRobot;
    private List<Pose3d> pose;
    private Pose3d sum;

    public CameraCalibration(Limelight limelight, Pose3d tagToRobot) {
        this.limelight = limelight;
        this.tagToRobot = tagToRobot;
        this.sum = new Pose3d();
    }

    public void addToPoseList() {
        pose.add(LimelightCalculations.getCameraToRobot(LimelightHelpers.getTargetPose3d_CameraSpace(limelight.getName()), tagToRobot));
		sum.getTranslation().plus(pose.getLast().getTranslation());
		sum.getRotation().plus(pose.getLast().getRotation());
    }

    public Pose3d getAvgPose() {
        return sum.div(pose.size());
    }

    @Override
    public void execute() {
        addToPoseList();
        Logger.recordOutput("/cameraCalibration/cameraToRobotPose", getAvgPose());
    }

    @Override
    public boolean isFinished() {
        return pose.size() > 100;
    }

}
