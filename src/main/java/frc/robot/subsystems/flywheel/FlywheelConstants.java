package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class FlywheelConstants {

    private static final TalonFXConfiguration CONFIGURATION = new TalonFXConfiguration();
    private static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
            Volts.of(1).per(Seconds.of(1)),
            Volts.of(7),
            Seconds.of(10),
            (state) -> SignalLogger.writeString("state", state.toString())
    );

    private static final double REFRESH_HERTZ = 50;

    static {
        Slot0Configs PID_SLOT_0 = new Slot0Configs();
        PID_SLOT_0.kP = 0.05;
        CONFIGURATION.withSlot0(PID_SLOT_0);
    }

    public static Flywheel getFlywheel(String logPath) {
        TalonFXWrapper rightMotorWrapper = new TalonFXWrapper(IDs.TalonFXs.RIGHT_FLYWHEEL);
        rightMotorWrapper.applyConfiguration(CONFIGURATION);

        TalonFXWrapper leftMotorWrapper = new TalonFXWrapper(IDs.TalonFXs.LEFT_FLYWHEEL);
        rightMotorWrapper.applyConfiguration(CONFIGURATION);

        return new Flywheel(
                logPath,
                new TalonFXMotor(
                        logPath + "right/",
                        rightMotorWrapper,
                        SYSID_CONFIG
                ),
                new TalonFXMotor(
                        logPath + "left/",
                        leftMotorWrapper,
                        SYSID_CONFIG
                ),
                Phoenix6SignalBuilder.generatePhoenix6Signal(rightMotorWrapper.getVelocity(), REFRESH_HERTZ, AngleUnit.ROTATIONS),
                Phoenix6SignalBuilder.generatePhoenix6Signal(leftMotorWrapper.getVelocity(), REFRESH_HERTZ, AngleUnit.ROTATIONS),
                new Phoenix6AngleRequest(new VelocityVoltage(0)),
                new Phoenix6AngleRequest(new VelocityVoltage(0)),
                new InputSignal[]{},
                new InputSignal[]{}
        );
    }


}
