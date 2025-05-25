package frc.robot.subsystems.swerve.module.factory;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleRequests;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class ModuleFactory {

    public static Module build(RealModuleConstants constants){

        String finalLogPath = ModuleConstants.LOG_PATH_PREFIX + constants.logPath();

        SysIdCalibrator.SysIdConfigInfo configInfo = new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), constants.isCTRE());

        TalonFXMotor drive = generateDrive(finalLogPath, constants);
        TalonFXMotor steer = generateSteer(finalLogPath, constants);
        IAngleEncoder encoder = generateEncoder(finalLogPath, constants);

        ModuleRequests requests = generateRequests();

        return new Module(finalLogPath, drive, steer, encoder, requests, , configInfo, constants);
    }

    private static TalonFXMotor generateDrive(String logPath, RealModuleConstants constants){
        return new TalonFXMotor(logPath + "/Drive", new Phoenix6DeviceID(constants.driveMotorId(), BusChain.ROBORIO), new SysIdRoutine.Config());
    }

    private static TalonFXMotor generateSteer(String logPath, RealModuleConstants constants){
        return new TalonFXMotor(logPath + "/Steer", new Phoenix6DeviceID(constants.steerMotorId(), BusChain.ROBORIO), new SysIdRoutine.Config());
    }

    private static IAngleEncoder generateEncoder(String logPath, RealModuleConstants constants){
        return new CANCoderEncoder(logPath + "/Encoder", new CANcoder(constants.encoderId(), BusChain.ROBORIO.getChainName()));
    }

    private static ModuleRequests generateRequests(){
        IRequest<Rotation2d> driveVelocityRequest = Phoenix6RequestBuilder.build(new VelocityVoltage(0), 0, ModuleConstants.enableFOC);
        IRequest<Double> driveVoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), ModuleConstants.enableFOC);
        IRequest<Rotation2d> steerAngleRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0), 0, ModuleConstants.enableFOC);

        return new ModuleRequests(driveVelocityRequest, driveVoltageRequest, steerAngleRequest);
    }

}
