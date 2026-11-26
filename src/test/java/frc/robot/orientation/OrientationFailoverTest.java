package frc.robot.orientation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.signal.MultiSampleAngleSignal;
import frc.utils.AngleUnit;
import frc.utils.Filter;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

class OrientationFailoverTest {

	private static class StubOrientationSupplier implements OrientationSupplier {

		private final MultiSampleAngleSignal yawSignal = new MultiSampleAngleSignal("TestYaw", AngleUnit.RADIANS);
		private Rotation3d orientation = new Rotation3d();
		private boolean reliable = true;

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
			Rotation2d[] yaws = yawSignal.asArray();
			Rotation3d[] result = new Rotation3d[yaws.length];
			for (int i = 0; i < yaws.length; i++) {
				result[i] = new Rotation3d(orientation.getX(), orientation.getY(), yaws[i].getRadians());
			}
			return result;
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
			this.yawSignal.setLatestValue(orientation.toRotation2d(), 0);
		}

		@Override
		public void updateOrientationSupplier() {}

	}

	@Test
	void multiSampleAngleSignalStoresAllSamplesAndTimestamps() {
		MultiSampleAngleSignal signal = new MultiSampleAngleSignal("RawYaw", AngleUnit.RADIANS);
		Rotation2d[] values = {Rotation2d.fromDegrees(10), Rotation2d.fromDegrees(25), Rotation2d.fromDegrees(40)};
		double[] timestamps = {1.0, 1.004, 1.008};

		signal.setValues(values, timestamps);

		assertArrayEquals(values, signal.asArray());
		assertArrayEquals(timestamps, signal.getTimestamps());
		assertEquals(Rotation2d.fromDegrees(40), signal.getLatestValue());
		assertEquals(1.008, signal.getTimestamp());

		signal.setValues(new Rotation2d[] {Rotation2d.fromDegrees(90)}, new double[] {1.0, 2.0});
		assertArrayEquals(values, signal.asArray());
	}

	@Test
	void syncsSwerveYawOnImuFailoverAndHandlesAllUnreliable() {
		StubOrientationSupplier imu = new StubOrientationSupplier();
		StubOrientationSupplier swerve = new StubOrientationSupplier();
		imu.setOrientation(new Rotation3d(0.1, -0.05, Math.toRadians(72)));
		swerve.setYaw(Rotation2d.fromDegrees(12));

		SuperDuperMegaOrientationSupplierFinalBoss switcher = new SuperDuperMegaOrientationSupplierFinalBoss(imu, swerve);
		assertEquals(Math.toRadians(72), switcher.getLatestOrientation().getZ(), 1e-6);

		// Simulate setHeading right before IMU drops (stale signal on immediate update)
		switcher.setYaw(Rotation2d.fromDegrees(135));
		imu.orientation = new Rotation3d(0.1, -0.05, Math.toRadians(72));
		switcher.updateOrientationSupplier();

		// IMU disconnects -> swerve receives synced yaw (135 deg) with roll/pitch preserved as 0
		imu.reliable = false;
		switcher.updateOrientationSupplier();

		assertEquals(Math.toRadians(135), swerve.getLatestOrientation().getZ(), 1e-6);
		assertEquals(0.0, swerve.getLatestOrientation().getX(), 1e-6);
		assertEquals(0.0, swerve.getLatestOrientation().getY(), 1e-6);
		assertEquals(Math.toRadians(135), switcher.getLatestOrientation().getZ(), 1e-6);

		// Both unreliable -> does not throw and reports unreliable
		swerve.reliable = false;
		assertDoesNotThrow(switcher::updateOrientationSupplier);
		assertFalse(switcher.isDataReliable().passesFilter());
	}

}
