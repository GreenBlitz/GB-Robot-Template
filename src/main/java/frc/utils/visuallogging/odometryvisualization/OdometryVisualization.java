package frc.utils.visuallogging.odometryvisualization;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.Supplier;

public class OdometryVisualization {

	public final String logPath;
	public final Supplier<Translation2d> deltaA;
	public final Supplier<Translation2d> deltaB;
	public final Supplier<Translation2d> deltaC;
	public final Supplier<Translation2d> deltaD;
	public final GridPlacements gridPlacements;

	public OdometryVisualization(
		String logPath,
		Supplier<Translation2d> deltaA,
		Supplier<Translation2d> deltaB,
		Supplier<Translation2d> deltaC,
		Supplier<Translation2d> deltaD,
		GridPlacements gridPlacements
	) {
		this.logPath = logPath;
		this.deltaA = deltaA;
		this.deltaB = deltaB;
		this.deltaC = deltaC;
		this.deltaD = deltaD;
		this.gridPlacements = gridPlacements;
	}

	public OdometryVisualization(String logPath, Supplier<SwerveModulePosition[]> modulePositions, GridPlacements gridPlacements) {
		this(
			logPath,
			() -> modulePositionToTranslation(modulePositions.get()[0]),
			() -> modulePositionToTranslation(modulePositions.get()[1]),
			() -> modulePositionToTranslation(modulePositions.get()[2]),
			() -> modulePositionToTranslation(modulePositions.get()[3]),
			gridPlacements
		);
	}

	private static Translation2d modulePositionToTranslation(SwerveModulePosition modulePosition) {
		// polar to cartezian
		return new Translation2d(modulePosition.distanceMeters, modulePosition.angle); // TODO: check units
	}

	private Translation2d getAverageChange() {
		return deltaA.get().plus(deltaB.get()).plus(deltaC.get()).plus(deltaD.get()).div(4d);
	}

	private Translation2d getAbsoluteCenter() {
		return gridPlacements.TOP_LEFT()
			.plus(gridPlacements.TOP_RIGHT())
			.plus(gridPlacements.BOTTOM_RIGHT())
			.plus(gridPlacements.BOTTOM_LEFT())
			.div(4d);
	}

	private GridPlacements getPlacementsAfterChange() {
		return this.gridPlacements.plus(new GridPlacements(deltaA.get(), deltaB.get(), deltaC.get(), deltaD.get()))
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
