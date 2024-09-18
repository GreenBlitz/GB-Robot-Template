package frc.robot.hardware.motor.phoenix6;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.CTREDevice;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.ControlAble;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;

public class TalonFXMotor extends CTREDevice implements IMotor, ControlAble {

    protected final TalonFXWrapper motor;
//    protected final SysIdCalibrator.SysIdConfigInfo sysidConfigInfo;

    public TalonFXMotor(String logPath, TalonFXWrapper motor, SysIdRoutine.Config sysidConfig) {
        super(logPath);
        this.motor = motor;
//        this.sysidConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, true);
    }

//    @Override
//    public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
//        return sysidConfigInfo;
//    }

    @Override
    public void setBrake(boolean brake) {
        NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.setNeutralMode(neutralModeValue);
    }

    @Override
    public void resetPosition(Rotation2d position) {
        motor.setPosition(position.getRotations());
    }


    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }


    @Override
    public void applyDoubleRequest(IRequest<Double> request) {
        if (request instanceof Phoenix6DoubleRequest) {
            motor.setControl(((Phoenix6DoubleRequest) request).getControlRequest());
        }
    }

    @Override
    public void applyAngleRequest(IRequest<Rotation2d> request) {
        if (request instanceof Phoenix6AngleRequest) {
            motor.setControl(((Phoenix6AngleRequest) request).getControlRequest());
        }
    }

}
