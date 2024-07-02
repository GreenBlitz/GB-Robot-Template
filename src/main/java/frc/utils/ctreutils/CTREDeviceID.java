package frc.utils.ctreutils;


public record CTREDeviceID(int ID, BusChain busChain) {

    public CTREDeviceID(int ID) {
        this(ID, BusChain.CANBUS);
    }

}
