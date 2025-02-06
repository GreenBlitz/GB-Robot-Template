package frc.robot.hardware.phoenix6;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class Phoenix6Util {

	public interface StatusCodeSupplier {

		StatusCode get();

	}

	public interface ErrorCodeSupplier {

		ErrorCode get();

	}

	public static <T> StatusSignal<T> getRefreshedSignal(boolean refresh, StatusSignal<T> signal) {
		return refresh ? signal.refresh() : signal;
	}

	public static StatusCode checkWithRetry(StatusCodeSupplier statusCodeSupplier, int numberOfTries) {
		for (int i = 0; i < numberOfTries - 1; i++) {
			if (statusCodeSupplier.get() == StatusCode.OK) {
				return StatusCode.OK;
			}
		}
		return statusCodeSupplier.get();
	}

	public static ErrorCode checkWithRetry(ErrorCodeSupplier errorCodeSupplier, int numberOfTries) {
		for (int i = 0; i < numberOfTries - 1; i++) {
			if (errorCodeSupplier.get() == ErrorCode.OK) {
				return ErrorCode.OK;
			}
		}
		return errorCodeSupplier.get();
	}

}
