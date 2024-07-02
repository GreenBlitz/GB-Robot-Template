package frc.robot.constants;

public class Ports {

    public enum JoystickPorts {

        MAIN(0),
        SECOND(1),
        THIRD(2),
        FOURTH(3),
        FIFTH(4),
        SIXTH(5);

        private final int port;

        JoystickPorts(int port) {
            this.port = port;
        }

        public int getPort() {
            return port;
        }

    }

}
