package frc.utils.ctreutils;

import com.ctre.phoenix6.StatusSignal;

public class PhoenixProUtils {

    public static <T> StatusSignal<T> getRefreshedSignal(boolean refresh, StatusSignal<T> signal) {
        return refresh ? signal.refresh() : signal;
    }

}
