package frc.robot.orientation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.Filter;

public class SuperDuperMegaOrientationSupplierFinalBoss implements OrientationSupplier {

    private final OrientationSupplier[] orientationSuppliers;
    private OrientationSupplier currentOrientationSupplier;
    private OrientationSource currentOrientationSource;

    public SuperDuperMegaOrientationSupplierFinalBoss(OrientationSupplier... orientationSuppliers) {
        this.orientationSuppliers = orientationSuppliers;
    }

    private OrientationSupplier getCurrentOrientationSupplier(){
        return
    }

    @Override
    public InputSignal<Rotation2d> getYawSignal() {
        return null;
    }

    @Override
    public Rotation3d getLatestOrientation() {
        return null;
    }

    @Override
    public Rotation3d[] getAllOrientations() {
        return new Rotation3d[0];
    }

    @Override
    public Filter isDataReliable() {
        return null;
    }

    @Override
    public void setYaw(Rotation2d yaw) {

    }

    @Override
    public void setOrientation(Rotation3d orientation) {

    }

    private void updateOrientationSource() {
        int best = orientationSuppliers.length;
        for (int i = 0; i < orientationSuppliers.length; i++) {
            if (orientationSuppliers[i].isDataReliable().passesFilter() && i < best) {
                best = i;
            }
        }
        currentOrientationSource = OrientationSource.IMU
    }

    @Override
    public void updateOrientationSupplier() {
        this.currentOrientationSupplier = orientationSuppliers[currentOrientationSource.getIndex()];
    }

}
