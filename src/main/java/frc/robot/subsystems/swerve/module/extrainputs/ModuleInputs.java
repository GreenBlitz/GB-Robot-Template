package frc.robot.subsystems.swerve.module.extrainputs;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleInputs {

	public boolean isClosedLoop = true;
	public SwerveModuleState targetState = new SwerveModuleState();

}
