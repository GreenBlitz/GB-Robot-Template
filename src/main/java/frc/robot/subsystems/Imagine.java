package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.rev.request.SparkMaxRequestBuilder;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.utils.AngleUnit;

public class Imagine extends GBSubsystem{

    private final BrushlessSparkMAXMotor motor;
    private final SuppliedAngleSignal position;
    private final SparkMaxWrapper wrapper;

    public Imagine(String logPath) {
        super(logPath);
        SimpleMotorSimulation motorSimulation = new SimpleMotorSimulation(
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getNEO(1),
                                1,
                                1
                        ),
                        DCMotor.getNEO(1)
                )
        );
        wrapper = new SparkMaxWrapper(new SparkMaxDeviceID(1));
        motor = new BrushlessSparkMAXMotor(logPath, wrapper, motorSimulation, new SysIdRoutine.Config());

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.p(10);
        wrapper.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        position = new SuppliedAngleSignal(logPath + "pos", () -> wrapper.getEncoder().getPosition(), AngleUnit.ROTATIONS);
    }

    public void setPosotiom(Rotation2d posotiom){
        motor.applyRequest(
                SparkMaxRequestBuilder.build(
                        posotiom,
                        SparkBase.ControlType.kPosition,
                        0
                )
        );
    }

    @Override
    protected void subsystemPeriodic() {
        motor.updateSimulation();
        motor.updateInputs(position);
    }
}
