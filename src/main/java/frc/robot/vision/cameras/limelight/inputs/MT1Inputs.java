package frc.robot.vision.cameras.limelight.inputs;

import edu.wpi.first.math.geometry.Pose3d;
import frc.utils.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MT1Inputs {

	public LimelightHelpers.PoseEstimate mtRawData = new LimelightHelpers.PoseEstimate();

    public Pose3d primaryTagPoseInCameraSpace = new Pose3d();

}
