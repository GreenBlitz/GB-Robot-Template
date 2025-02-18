package frc.robot.hardware.phoenix6;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import java.util.function.Supplier;

public class Phoenix6Util {

	public static <T> StatusSignal<T> getRefreshedSignal(boolean refresh, StatusSignal<T> signal) {
		return refresh ? signal.refresh() : signal;
	}

	public static StatusCode checkStatusCodeWithRetry(Supplier<StatusCode> statusCodeSupplier, int numberOfTries) {
		for (int i = 0; i < numberOfTries - 1; i++) {
			if (statusCodeSupplier.get().isOK()) {
				return StatusCode.OK;
			}
		}
		return statusCodeSupplier.get();
	}

	public static ErrorCode checkErrorCodeWithRetry(Supplier<ErrorCode> errorCodeSupplier, int numberOfTries) {
		for (int i = 0; i < numberOfTries - 1; i++) {
			if (errorCodeSupplier.get() == ErrorCode.OK) {
				return ErrorCode.OK;
			}
		}
		return errorCodeSupplier.get();
	}

}
