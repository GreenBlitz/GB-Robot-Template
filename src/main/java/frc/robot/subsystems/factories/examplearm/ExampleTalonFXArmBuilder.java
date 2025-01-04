package frc.robot.subsystems.factories.examplearm;


import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.motors.TalonFXWrapper;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.subsystems.arm.ExampleArm;
import frc.utils.AngleUnit;

import static frc.robot.IDs.TalonFXIDs.ARM_DEVICE_ID;

public class ExampleTalonFXArmBuilder {
    static ExampleArm build(String logPath){
        Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0).withSlot(0).withEnableFOC(true));
        Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true));

        Phoenix6DeviceID deviceID = new Phoenix6DeviceID(ARM_DEVICE_ID.id());
        SysIdRoutine.Config config = new SysIdRoutine.Config();
        TalonFXMotor motor = new TalonFXMotor(logPath, deviceID, config);
        Phoenix6AngleSignal positionSignal = new Phoenix6AngleSignal("Arm position", null, AngleUnit.ROTATIONS);
    }

}
