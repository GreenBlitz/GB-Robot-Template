package frc.constants;

import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtils;

public class Field {

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	public static final double LENGTH_METERS = ;
	public static final double WIDTH_METERS = ;

}
