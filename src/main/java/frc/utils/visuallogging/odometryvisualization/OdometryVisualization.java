package frc.utils.visuallogging.odometryvisualization;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Supplier;

public class OdometryVisualization {

	public final String logPath;
	public final Supplier<Transform2d> deltaA;
	public final Supplier<Transform2d> deltaB;
	public final Supplier<Transform2d> deltaC;
	public final Supplier<Transform2d> deltaD;
	public final GridPlacements gridPlacements;

	public OdometryVisualization(
		String logPath,
		Supplier<Transform2d> deltaA,
		Supplier<Transform2d> deltaB,
		Supplier<Transform2d> deltaC,
		Supplier<Transform2d> deltaD,
		GridPlacements gridPlacements
	) {
		this.logPath = logPath;
		this.deltaA = deltaA;
		this.deltaB = deltaB;
		this.deltaC = deltaC;
		this.deltaD = deltaD;
		this.gridPlacements = gridPlacements;
	}

	private Translation2d getAverageChange() {
		return deltaA.get().plus(deltaB.get()).plus(deltaC.get()).plus(deltaD.get()).div(4d).getTranslation();
	}

	private Translation2d getAbsoluteCenter() {
		return gridPlacements.TOP_LEFT()
			.plus(gridPlacements.TOP_RIGHT())
			.plus(gridPlacements.BOTTOM_RIGHT())
			.plus(gridPlacements.BOTTOM_LEFT())
			.div(4d);
	}

	private GridPlacements getPlacementsAfterChange() {
		return this.gridPlacements
			.plus(
				new GridPlacements(
					deltaA.get().getTranslation(),
					deltaB.get().getTranslation(),
					deltaC.get().getTranslation(),
					deltaD.get().getTranslation()
				)
			)
			.plus(new GridPlacements(getAbsoluteCenter().unaryMinus()));
	}

	public void drawCorners() {
		gridPlacements.log(this.logPath + "/beforeChange/");
	}

	public void drawCornersAfterChange() {
		getPlacementsAfterChange().log(this.logPath + "/afterChange/");
	}

	public void drawAll() {
		drawCorners();
		drawCornersAfterChange();
	}

}
