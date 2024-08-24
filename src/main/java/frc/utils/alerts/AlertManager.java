package frc.utils.alerts;

import java.util.ArrayList;

public class AlertManager {

	private static ArrayList<PeriodicAlert> alertList = new ArrayList<>();

	public static void addAlert(PeriodicAlert periodicAlert) {
		alertList.add(periodicAlert);
	}

	public static void reportAlerts() {
		for (PeriodicAlert alert : alertList) {
			alert.reportByCondition();
		}
	}

}
