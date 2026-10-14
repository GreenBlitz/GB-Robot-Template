package frc.robot.subsystems.swerve.module.extrainputs;

import org.wpilib.math.kinematics.SwerveModuleVelocity;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ModuleInputs {

	public boolean isClosedLoop = ModuleConstants.DEFAULT_IS_CLOSE_LOOP;
	public SwerveModuleVelocity targetVelocity = new SwerveModuleVelocity();
	public String controlMode = ModuleUtil.ControlMode.NONE.toLog();

}
