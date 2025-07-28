package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.utils.filter.Filter;
import frc.utils.math.ToleranceMath;

import java.util.function.Supplier;

public class LimelightFilters {

	public Filter megaTag1Filter(Limelight limelight) {
		return Filter.nonFilteringFilter();
	}

	public Filter megaTag2Filter(Limelight limelight) {
		return Filter.nonFilteringFilter();
	}

	private Filter isRobotInField(Supplier<Translation2d> robotTranslation) {
		return () -> ToleranceMath.isInRange(robotTranslation.get().getX(), 0, Field.LENGTH_METERS)
			&& ToleranceMath.isInRange(robotTranslation.get().getY(), 0, Field.WIDTH_METERS);
	}

}
