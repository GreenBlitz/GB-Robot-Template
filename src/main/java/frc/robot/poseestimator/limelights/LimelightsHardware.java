package frc.robot.poseestimator.limelights;

import frc.utils.GBSubsystem;

import java.util.ArrayList;
import java.util.List;

public class LimelightsHardware extends GBSubsystem {

	private List<Limelight> limelights;


	public LimelightsHardware(String[] limelightNames, String hardwareLogpath) {
		super(hardwareLogpath);

		this.limelights = new ArrayList<>();
		for (String limelightName : limelightNames) {
			limelights.add(new Limelight(limelightName, hardwareLogpath));
		}
	}

	public List<Limelight> getAllLimelights() {
		return limelights;
	}

	@Override
	protected void subsystemPeriodic() {}


}
