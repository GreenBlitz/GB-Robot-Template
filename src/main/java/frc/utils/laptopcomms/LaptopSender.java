package frc.utils.laptopcomms;



public class LaptopSender {

    public static final String ROBOT_IP = "10.45.90.183";
    public static final String ROBOTICA_IP = "192.168.1.87";
    public static final int PORT = 8082;

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

    }

    private static void laptopPeriodic() {

    }
}
