package frc.utils.alerts;

import java.util.function.BooleanSupplier;

public class PeriodicAlert extends Alert {

	private final BooleanSupplier reportSupplier;

	public PeriodicAlert(AlertType type, String name, BooleanSupplier reportSupplier) {
		super(type, name);
		this.reportSupplier = reportSupplier;
	}

	protected void reportByCondition() {
		if (reportSupplier.getAsBoolean()) {
			report();
		}
	}

}
