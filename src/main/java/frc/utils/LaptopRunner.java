package frc.utils;

import frc.utils.controllers.keyboard.KeyboardListener;

public class LaptopRunner {

    public static final boolean ENABLE_KEYBOARD = true;

    public static void main(String[] args) {
        laptopStart();
    }

    private static void laptopLogic() {
        laptopStart();
        try {
            while (true) {
                laptopPeriodic();
                Thread.sleep(20);
            }
        } catch (Exception e) {
            System.out.println("Laptop Periodic Stopped:  " + e);
        }
    }

    private static void laptopStart() {
        KeyboardListener.startTrackingKeyboard();
    }

    private static void laptopPeriodic() {

    }
}
