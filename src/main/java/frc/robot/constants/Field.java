package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtils;

public class Field {

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	public static final double LENGTH_METERS = 16.54175;

	public static final double WIDTH_METERS = 8.0137;

	public final static double APRIL_TAG_HEIGHT_METERS = 1.2397;

}
