package frc.utils.alerts;

import java.util.function.BooleanSupplier;

public class PeriodicAlert extends Alert {

	private final BooleanSupplier raiseSupplier;

	public PeriodicAlert(AlertType type, String name, BooleanSupplier raiseSupplier) {
		super(type, name);
		this.raiseSupplier = raiseSupplier;
		AlertManager.addToAlertList(this);
	}

	protected void periodic() {
		if (raiseSupplier.getAsBoolean()) {
			logAlert();
		}
	}

}
