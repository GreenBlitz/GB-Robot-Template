package frc.robot.subsystems.swerve.module.extrainputs;

import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

@AutoLog
public class ModuleInputs {

  public boolean isClosedLoop = ModuleConstants.DEFAULT_IS_CLOSE_LOOP;
  public SwerveModuleVelocity targetVelocity = new SwerveModuleVelocity();
  public String controlMode = ModuleUtil.ControlMode.NONE.toLog();
}
