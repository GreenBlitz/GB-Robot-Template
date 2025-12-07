package frc.robot.vision.cameras.limelight.inputs;

import frc.robot.vision.cameras.limelight.LimelightTarget2dValues;
import frc.utils.limelight.LimelightHelpersSupplements;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ColorDetectionInputs {

	public LimelightTarget2dValues target2dValues = new LimelightTarget2dValues();

	public LimelightHelpersSupplements.RawTarget[] rawTargets = new LimelightHelpersSupplements.RawTarget[0];

}
