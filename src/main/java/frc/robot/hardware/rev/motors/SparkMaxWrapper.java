package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;

public class SparkMaxWrapper extends SparkMax {

	public SparkMaxWrapper(SparkMaxDeviceID deviceID) {
		super(deviceID.id(), deviceID.type());

		// TODO super.configure()
//		super.restoreFactoryDefaults();
	}

	public double getVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}

	public Rotation2d getVelocityAnglePerSecond() {
		return Rotation2d.fromRotations(Conversions.perMinuteToPerSecond(getEncoder().getVelocity()));
	}

	public void reportWarnings(String logPath){
		Warnings warnings = getWarnings();

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "BrownoutAt",
						() -> warnings.brownout
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "OverCurrentAt",
						() -> warnings.overcurrent
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "EscEepromAt",
						() -> warnings.escEeprom
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "ExtEepromAt",
						() -> warnings.extEeprom
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "SensorAt",
						() -> warnings.sensor
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "StallAt",
						() -> warnings.stall
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "HasResetAt",
						() -> warnings.hasReset
				)
		);

		AlertManager.addAlert(
				new PeriodicAlert(
						Alert.AlertType.WARNING,
						logPath + "OtherAt",
						() -> warnings.other
				)
		);

	}

}
