package frc.robot.poseestimator.limelights;

import frc.utils.GBSubsystem;

import java.util.ArrayList;
import java.util.List;

public class Limelights extends GBSubsystem {

	private List<Limelight> limelights;


	public Limelights(String[] name, String hardwareLogPath) {
		super(hardwareLogPath);

		this.limelights = new ArrayList<>();
		for (String limelightName : name) {
			limelights.add(new Limelight(limelightName, hardwareLogPath));
		}
	}

	public List<Limelight> getAllLimelights() {
		return limelights;
	}

	@Override
	protected void subsystemPeriodic() {}


}
