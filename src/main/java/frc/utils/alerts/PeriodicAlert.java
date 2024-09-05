package frc.utils.alerts;

import java.util.function.BooleanSupplier;

public class PeriodicAlert extends Alert {

	private final BooleanSupplier reportCondition;

	public PeriodicAlert(AlertType type, String name, BooleanSupplier reportCondition) {
		super(type, name);
		this.reportCondition = reportCondition;
	}

	protected void reportByCondition() {
		if (reportCondition.getAsBoolean()) {
			report();
		}
	}

}
