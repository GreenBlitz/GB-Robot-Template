package frc.robot.hardware.phoenix6;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import java.util.function.Supplier;

public class PhoenixProUtils {

	public static <T> StatusSignal<T> getRefreshedSignal(boolean refresh, StatusSignal<T> signal) {
		return refresh ? signal.refresh() : signal;
	}

	public static StatusCode checkWithRetry(Supplier<StatusCode> statusCodeSupplier, int numberOfTries) {
		for (int i = 0; i < numberOfTries - 1; i++) {
			if (statusCodeSupplier.get().isOK()) {
				return StatusCode.OK;
			}
		}
		return statusCodeSupplier.get();
	}

}
