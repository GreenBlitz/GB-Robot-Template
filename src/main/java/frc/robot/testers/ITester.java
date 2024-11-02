package frc.robot.testers;

import com.ctre.phoenix6.controls.ControlRequest;

public interface ITester {

	void run();

	void setControl(ControlRequest controlRequest);

}
