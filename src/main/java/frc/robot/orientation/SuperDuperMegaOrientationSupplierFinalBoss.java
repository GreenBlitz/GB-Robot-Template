package frc.robot.orientation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.Filter;
import org.littletonrobotics.junction.Logger;

public class SuperDuperMegaOrientationSupplierFinalBoss implements OrientationSupplier {

	private final OrientationSupplier[] orientationSuppliers;
	private OrientationSupplier currentOrientationSupplier;
	private OrientationSource currentOrientationSource;

	public SuperDuperMegaOrientationSupplierFinalBoss(OrientationSupplier... orientationSuppliers) {
		this.orientationSuppliers = orientationSuppliers;
		updateOrientationSupplier();
	}

	private OrientationSupplier getCurrentOrientationSupplier() {
		return currentOrientationSupplier;
	}

	@Override
	public InputSignal<Rotation2d> getYawSignal() {
		return currentOrientationSupplier.getYawSignal();
	}

	@Override
	public Rotation3d getLatestOrientation() {
		return currentOrientationSupplier.getLatestOrientation();
	}

	@Override
	public Rotation3d[] getAllOrientations() {
		return currentOrientationSupplier.getAllOrientations();
	}

	@Override
	public Filter isDataReliable() {
		return currentOrientationSupplier.isDataReliable();
	}

	@Override
	public void setYaw(Rotation2d yaw) {
		currentOrientationSupplier.setYaw(yaw);
	}

	@Override
	public void setOrientation(Rotation3d orientation) {
		currentOrientationSupplier.setOrientation(orientation);
	}

	private void updateOrientationSource() {
		int best = orientationSuppliers.length;
		for (int i = 0; i < orientationSuppliers.length; i++) {
			if (orientationSuppliers[i].isDataReliable().passesFilter() && i < best) {
				best = i;
			}
		}
		currentOrientationSource = OrientationSource.getSourceByIndex(best);
	}

	@Override
	public void updateOrientationSupplier() {
		updateOrientationSource();

		if (currentOrientationSource == OrientationSource.NONE) {
			if (currentOrientationSupplier == null && orientationSuppliers.length > 0) {
				currentOrientationSupplier = orientationSuppliers[orientationSuppliers.length - 1];
			}
			Logger.recordOutput("Orientation/CurrentSource", currentOrientationSource);
			return;
		}

		OrientationSupplier nextSupplier = orientationSuppliers[currentOrientationSource.getIndex()];
		if (currentOrientationSupplier != null && currentOrientationSupplier != nextSupplier) {
			nextSupplier.setYaw(currentOrientationSupplier.getLatestOrientation().toRotation2d());
		}
		currentOrientationSupplier = nextSupplier;
		Logger.recordOutput("Orientation/CurrentSource", currentOrientationSource);
	}

}
