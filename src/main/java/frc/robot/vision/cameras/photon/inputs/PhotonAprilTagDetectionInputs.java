package frc.robot.vision.cameras.photon.inputs;


import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class PhotonAprilTagDetectionInputs {

	public Pose3d robotPoseAccordingToBestTarget;

	public double poseAmbiguity;

	public int bestTargetFiducialId;

	public double timestamp;

	public double latency;

	public boolean hasTargets;

	public double targetAreaWithinScreen;

	public double targetDistance;

}
