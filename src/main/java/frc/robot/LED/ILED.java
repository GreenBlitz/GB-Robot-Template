package frc.robot.LED;


import java.awt.*;

public interface ILED {

	void setSectionColor(Color color, int startIndex, int endIndex);

	void setSingleLEDColor(Color color, int index);

	void sectionTurnOff(int startIndex, int endIndex);

	void singleLEDTurnOff(int index);

	void update();

}
