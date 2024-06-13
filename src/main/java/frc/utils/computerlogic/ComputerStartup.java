package frc.utils.computerlogic;

public class ComputerStartup {

    public static void main(String[] args) {
        startupLogic();
    }

    public static void startupLogic() {
        if (CMDHandler.TEST) {
            CMDHandler.runCMDCommand("echo Hello World!");
        }
    }
}
