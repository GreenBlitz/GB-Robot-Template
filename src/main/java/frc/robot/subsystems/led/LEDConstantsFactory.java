package frc.robot.subsystems.led;
    public class LEDConstantsFactory {

        private LEDConstantsFactory() {
        }

        public static LEDConstants createConstants() {
            return new LEDConstants(
                    4,
                    50,
                    0.5,
                    10,
                    0.05,
                    255,
                    255
            );
        }
    }
