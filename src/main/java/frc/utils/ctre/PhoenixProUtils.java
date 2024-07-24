package frc.utils.ctre;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.function.Supplier;

public class PhoenixProUtils {

    public static <T> StatusSignal<T> getRefreshedSignal(boolean refresh, StatusSignal<T> signal) {
        return refresh ? signal.refresh() : signal;
    }

    public static boolean applyThing(Supplier<StatusCode> statusCodeSupplier, int tries){
        for (int i = 0; i < tries; i++){
            if(statusCodeSupplier.get().isOK()){
                return true;
            }
        }
        return false;
    }

}
