package frc.robot.hardware.phoenix6;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import java.util.function.Supplier;

public class Phoenix6Util {

	public static <T> StatusSignal<T> getRefreshedSignal(boolean refresh, StatusSignal<T> signal) {
		return refresh ? signal.refresh() : signal;
	}

//	what was before:
//	public static StatusCode checkWithRetry(Getter<StatusCode> statusCodeSupplier, int numberOfTries) {
//		for (int i = 0; i < numberOfTries - 1; i++) {
//			if (statusCodeSupplier.get().isOK()) {
//				return StatusCode.OK;
//			}
//		}
//		return statusCodeSupplier.get();
//	}
//
//	public static ErrorCode checkWithRetry(Getter<ErrorCode> errorCodeSupplier, int numberOfTries) {
//		for (int i = 0; i < numberOfTries - 1; i++) {
//			if (errorCodeSupplier.get() == ErrorCode.OK) {
//				return ErrorCode.OK;
//			}
//		}
//		return errorCodeSupplier.get();
//	}

// option one:
//	public static <T> T checkWithRetry(Supplier<T> tCodeSupplier, int numberOfTries) {
//		Class<?> aClass = tCodeSupplier.get().getClass();
//		if (aClass.equals(StatusCode.class)) {
//			for (int i = 0; i < numberOfTries - 1; i++) {
//				if (tCodeSupplier.get() == StatusCode.OK) {
//					return (T) StatusCode.OK;
//				}
//			}
//		} else if (aClass.equals(ErrorCode.class)) {
//			for (int i = 0; i < numberOfTries - 1; i++) {
//				if (tCodeSupplier.get() == ErrorCode.OK) {
//					return (T) ErrorCode.OK;
//				}
//			}
//		}
//		return tCodeSupplier.get();
//	}
//	option 2:
	public static <T> T checkWithRetry(Supplier<T> tCodeSupplier, int numberOfTries) {
		for (int i = 0; i < numberOfTries - 1; i++) {
			if (tCodeSupplier.get() == (tCodeSupplier.get() instanceof StatusCode ? StatusCode.OK : ErrorCode.OK)) {
				return (T) (tCodeSupplier.get() instanceof StatusCode ? StatusCode.OK : ErrorCode.OK);
			}
		}
		return tCodeSupplier.get();
	}

}
