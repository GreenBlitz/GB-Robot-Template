package frc.utils.alerts;

import java.util.function.BooleanSupplier;

public class PeriodicAlert extends Alert {

	private final BooleanSupplier raiseSupplier;
	private boolean wasLogged;

	public PeriodicAlert(AlertType type, String name, BooleanSupplier raiseSupplier) {
		super(type, name);
		this.raiseSupplier = raiseSupplier;
		AlertManager.addAlert(this);
		this.wasLogged = false;
	}

	protected void periodic() {
		if (raiseSupplier.getAsBoolean() && !wasLogged) {
			logAlert();
			wasLogged = true;
		}
	}

}
