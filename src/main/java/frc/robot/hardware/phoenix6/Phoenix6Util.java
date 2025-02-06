package frc.robot.hardware.phoenix6;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import java.util.function.Supplier;

public class Phoenix6Util {

	public static <T> StatusSignal<T> getRefreshedSignal(boolean refresh, StatusSignal<T> signal) {
		return refresh ? signal.refresh() : signal;
	}

	public static <T> T checkWithRetry(Supplier<T> tCodeSupplier, int numberOfTries) {
		for (int i = 0; i < numberOfTries - 1; i++) {
			if (tCodeSupplier.get() == (tCodeSupplier.get() instanceof StatusCode ? StatusCode.OK : ErrorCode.OK)) {
				return (T) (tCodeSupplier.get() instanceof StatusCode ? StatusCode.OK : ErrorCode.OK);
			}
		}
		return tCodeSupplier.get();
	}

}
