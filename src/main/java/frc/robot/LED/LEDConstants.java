package frc.robot.LED;

import edu.wpi.first.wpilibj.util.Color;
import org.w3c.dom.css.RGBColor;

public class LEDConstants {

	public class CANdle {

		public static final int ID_PORT = 0;
		public static final double BLINK_DURATION = 0.5;


	}

	public class LEDStrip {

		public static final int LED_LENGTH = 200;
		public static final int LED_PORT = 9;
		public static final double BLINK_DURATION = 0.5;
		public static final Color BLINK_COLOR = Color.kDarkGreen;
		public static final Color COLORFLOW_ANIMATION_COLOR = Color.kMediumPurple;
		public static final Color LARSON_ANIMATION_COLOR = Color.kAqua;



	}

	public class LEDFactory {

		public static final boolean LED_IS_CANDLE = false;

	}

}
