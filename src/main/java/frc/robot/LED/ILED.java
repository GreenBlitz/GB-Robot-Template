package frc.robot.LED;

import java.awt.*;
import edu.wpi.first.wpilibj.util.Color;


public interface ILED {

	void setColor(Color color, int index);

	void turnOff(int index);

}
