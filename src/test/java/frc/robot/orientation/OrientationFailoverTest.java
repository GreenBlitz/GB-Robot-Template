package frc.robot.orientation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.utils.AngleUnit;
import frc.utils.Filter;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

class OrientationFailoverTest {

	private static class StubOrientationSupplier implements OrientationSupplier {

		private final SuppliedAngleSignal yawSignal;
		private Rotation3d orientation = new Rotation3d();
		private boolean reliable = true;

		StubOrientationSupplier() {
			this.yawSignal = new SuppliedAngleSignal("TestYaw", () -> orientation.getZ(), AngleUnit.RADIANS);
		}

		@Override
		public InputSignal<Rotation2d> getYawSignal() {
			return yawSignal;
		}

		@Override
		public Rotation3d getLatestOrientation() {
			return orientation;
		}

		@Override
		public Rotation3d[] getAllOrientations() {
			return new Rotation3d[] {orientation};
		}

		@Override
		public Filter isDataReliable() {
			return () -> reliable;
		}

		@Override
		public void setYaw(Rotation2d yaw) {
			setOrientation(new Rotation3d(orientation.getX(), orientation.getY(), yaw.getRadians()));
		}

		@Override
		public void setOrientation(Rotation3d orientation) {
			this.orientation = orientation;
		}

		@Override
		public void updateOrientationSupplier() {}

	}

	@Test
	void syncsSwerveYawOnImuFailoverAndHandlesAllUnreliable() {
		StubOrientationSupplier imu = new StubOrientationSupplier();
		StubOrientationSupplier swerve = new StubOrientationSupplier();
		imu.setOrientation(new Rotation3d(0.1, -0.05, Math.toRadians(72)));
		swerve.setYaw(Rotation2d.fromDegrees(12));

		SuperDuperMegaOrientationSupplierFinalBoss switcher = new SuperDuperMegaOrientationSupplierFinalBoss(imu, swerve);
		assertEquals(Math.toRadians(72), switcher.getLatestOrientation().getZ(), 1e-6);

		// IMU disconnects -> swerve receives synced yaw from IMU's last orientation
		imu.reliable = false;
		switcher.updateOrientationSupplier();

		assertEquals(Math.toRadians(72), swerve.getLatestOrientation().getZ(), 1e-6);
		assertEquals(Math.toRadians(72), switcher.getLatestOrientation().getZ(), 1e-6);

		// Both unreliable -> does not throw and reports unreliable
		swerve.reliable = false;
		assertDoesNotThrow(switcher::updateOrientationSupplier);
		assertFalse(switcher.isDataReliable().passesFilter());
	}

}
