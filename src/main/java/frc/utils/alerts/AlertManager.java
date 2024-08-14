package frc.utils.alerts;

import java.util.ArrayList;

public class AlertManager {

	private static ArrayList<PeriodicAlert> alertList = new ArrayList<>();

	protected static void addToAlertList(PeriodicAlert periodicAlert) {
		alertList.add(periodicAlert);
	}

	public static void periodic() {
		for (PeriodicAlert alert : alertList) {
			alert.periodic();
		}
	}

}
