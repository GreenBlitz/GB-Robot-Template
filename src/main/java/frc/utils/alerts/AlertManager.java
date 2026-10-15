package frc.utils.alerts;

import java.util.ArrayList;

public class AlertManager {

	private static final ArrayList<PeriodicAlert> periodicAlerts = new ArrayList<>();
	private static final ArrayList<Alert> reportedAlerts = new ArrayList<>();

	public static void addAlert(PeriodicAlert periodicAlert) {
		periodicAlerts.add(periodicAlert);
	}

	public static void reportAlerts() {
		for (PeriodicAlert alert : periodicAlerts) {
			alert.reportByCondition();
		}
	}

	public static ArrayList<PeriodicAlert> getPeriodicAlerts() {
		return periodicAlerts;
	}

	public static ArrayList<Alert> getReportedAlerts() {
		return reportedAlerts;
	}

}
