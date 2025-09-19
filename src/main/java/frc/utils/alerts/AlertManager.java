package frc.utils.alerts;

import java.util.ArrayList;
import java.util.Collection;

public class AlertManager {

	private static final Collection<PeriodicAlert> alerts = new ArrayList<>();

	public static void addAlert(PeriodicAlert periodicAlert) {
		alerts.add(periodicAlert);
	}

	public static void reportAlerts() {
		for (PeriodicAlert alert : alerts) {
			alert.reportByCondition();
		}
	}

}
