package frc.robot.poseestimator.limelight;

import frc.utils.GBSubsystem;

import java.util.ArrayList;
import java.util.List;

public class LimelightsHardware extends GBSubsystem {

	private List<Limelight> limelights;


	public LimelightsHardware(String[] limelightNames, String limelightManagerName) {
		super(limelightManagerName + "Hardware/");

		limelights = new ArrayList<>();
		for (String limelightName : limelightNames) {
			limelights.add(new Limelight(limelightName));
		}
	}

	public List<Limelight> getAllLimelights() {
		return limelights;
	}

	@Override
	protected void subsystemPeriodic() {}

}
