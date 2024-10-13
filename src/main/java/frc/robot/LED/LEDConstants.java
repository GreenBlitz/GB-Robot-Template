package frc.robot.LED;


import java.awt.*;

public class LEDConstants {

	public class CANdle {

		public static final int ID_PORT = 0;
		public static final double BLINK_DURATION = 0.5;


	}

	public class LEDStrip {

		public static final int LED_LENGTH = 200;
		public static final int LED_PORT = 4;
		public static final double BLINK_DURATION = 0.5;
		public static final Color BLINK_COLOR = Color.GREEN;
		public static final Color COLORFLOW_ANIMATION_COLOR = Color.PINK;
		public static final Color LARSON_ANIMATION_COLOR = Color.CYAN;
		public static final double LARSON_ANIMATION_SPEED = 2.5;
		public static final double COLORFLOW_ANIMATION_SPEED = 2.5;


	}

	public class LEDFactory {

		public static final boolean LED_IS_CANDLE = false;

	}

}
