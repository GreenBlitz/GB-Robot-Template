package frc.robot.vision.cameras.limelight.inputs;

import frc.robot.vision.cameras.limelight.LimelightTarget2dValues;
import frc.utils.limelight.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class NeuralDetectionInputs {

	public LimelightTarget2dValues target2dValues = new LimelightTarget2dValues();

	public LimelightHelpers.RawDetection[] rawDetections = new LimelightHelpers.RawDetection[] {};

}
