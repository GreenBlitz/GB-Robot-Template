package frc.utils.ctreutils;


public class BusStatus {

    protected static final double MAX_CAN_UTILIZATION_PERCENT = 0.6;
    protected static final double MAX_RECEIVE_ERRORS = 0;
    protected static final double MAX_TRANSMIT_ERRORS = 0;
    protected static final String LOG_PATH = "Bus/";

    public static void logChainsStatuses() {
        for (BusChain chain : BusChain.values()) {
            chain.updateStatus();
        }
    }

}
