package frc.robot.hardware.rev;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;

import java.util.function.Supplier;

public class REVUtils {

    public static REVLibError checkWithRetry(Supplier<REVLibError> revLibErrorSupplier, int numberOfTries) {
        for (int i = 0; i < numberOfTries - 1; i++) {
            if (revLibErrorSupplier.get() == REVLibError.kOk) {
                return REVLibError.kOk;
            }
        }
        return revLibErrorSupplier.get();
    }

}
