package frc.utils.visuallogging.odometryvisualization;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public record GridPlacements(Translation2d TOP_LEFT, Translation2d TOP_RIGHT, Translation2d BOTTOM_LEFT, Translation2d BOTTOM_RIGHT) {

	public GridPlacements(Translation2d translation) {
		this(translation, translation, translation, translation);
	}

	public GridPlacements plus(GridPlacements other) {
		return new GridPlacements(
			this.TOP_LEFT.plus(other.TOP_LEFT),
			this.TOP_RIGHT.plus(other.TOP_RIGHT),
			this.BOTTOM_LEFT.plus(other.BOTTOM_LEFT),
			this.BOTTOM_RIGHT.plus(other.BOTTOM_RIGHT)
		);
	}

	public GridPlacements multiply(double factor) {
		return new GridPlacements(
			this.TOP_LEFT.times(factor),
			this.TOP_RIGHT.times(factor),
			this.BOTTOM_LEFT.times(factor),
			this.BOTTOM_RIGHT.times(factor)
		);
	}

	public void log(String logPath) {
		Logger.recordOutput(logPath + "/ExactTopLeft", this.TOP_LEFT());
		Logger.recordOutput(logPath + "/ExactTopRight", this.TOP_RIGHT());
		Logger.recordOutput(logPath + "/ExactBottomRight", this.BOTTOM_RIGHT());
		Logger.recordOutput(logPath + "/ExactBottomLeft", this.BOTTOM_LEFT());
	}

}
