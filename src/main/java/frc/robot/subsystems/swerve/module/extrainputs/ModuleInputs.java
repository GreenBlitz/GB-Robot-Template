package frc.robot.subsystems.swerve.module.extrainputs;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleInputs {

	public boolean isClosedLoop = ModuleConstants.DEFAULT_IS_CLOSE_LOOP;
	public SwerveModuleState targetState = new SwerveModuleState();

}
