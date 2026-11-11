package frc.robot.vision.cameras.limelight.inputs;

import frc.utils.limelight.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose3d;

@AutoLog
public class MTInputs {

  public LimelightHelpers.PoseEstimate mtRawData = new LimelightHelpers.PoseEstimate();

  public Pose3d primaryTagPoseInCameraSpace = new Pose3d();
}
