package frc.constants.field;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtil;

public class Field {

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtil.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	public static final double LENGTH_METERS = 16.54175;
	public static final double WIDTH_METERS = 8.0137;

	public static Pose2d getAllianceRelativePose(Pose2d conventionAlliancePose) {
		return isFieldConventionAlliance() ? conventionAlliancePose : FlippingUtil.flipFieldPose(conventionAlliancePose);
	}

}
