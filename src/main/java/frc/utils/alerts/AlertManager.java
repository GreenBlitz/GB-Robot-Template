package frc.utils.alerts;

import java.util.ArrayList;

public class AlertManager {

	private static final ArrayList<PeriodicAlert> alerts = new ArrayList<>();

	public static void addAlert(PeriodicAlert periodicAlert) {
		alerts.add(periodicAlert);
	}

	public static void reportAlerts() {
		for (PeriodicAlert alert : alerts) {
			alert.reportByCondition();
		}
	}

}
