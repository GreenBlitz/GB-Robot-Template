package frc.robot.LED;

import edu.wpi.first.wpilibj.util.Color;
import java.awt.*;

public interface ILED {

	void setColor(Color color, int index);

	void turnOff(int index);

}
