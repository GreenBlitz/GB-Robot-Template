package frc.robot.LED;

import edu.wpi.first.wpilibj.util.Color;

public interface ILED {

	void setSectionColor(Color color, int startIndex, int endIndex);
	void setSingleLEDColor(Color color, int index);

	void sectionTurnOff(int startIndex, int endIndex);

	void singleLEDTurnOff(int index);

}
