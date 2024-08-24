package frc.utils.alerts;

import java.util.function.BooleanSupplier;

public class PeriodicAlert extends Alert {

	private final BooleanSupplier raiseSupplier;

	public PeriodicAlert(AlertType type, String name, BooleanSupplier raiseSupplier) {
		super(type, name);
		this.raiseSupplier = raiseSupplier;
	}

	protected void reportByCondition() {
		if (raiseSupplier.getAsBoolean()) {
			reportAlert();
		}
	}

}
