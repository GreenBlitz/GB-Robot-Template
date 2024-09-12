package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.hardware.request.IRequest;

import java.util.function.Consumer;

public class CANSparkMAXRequest implements IRequest<Double> {

    void AAA(){
        CANSparkMax can = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushed);
        can.getPIDController().setReference(5, CANSparkBase.ControlType.kVoltage, 0);
    }

    private Consumer<Double> withSetPoint;
    private CANSparkBase.ControlType controlType;
    private int pidSlot;

    private CANSparkMAXRequest(Consumer<Double> withSetPoint, CANSparkBase.ControlType controlType, int pidSlot){
        this.withSetPoint = withSetPoint;
        this.controlType = controlType;
        this.pidSlot = pidSlot;
    }

//    public CANSparkMAXRequest(Double positionSetPoint, int pidSlot){
//        this(positionSetPoint -> {}, CANSparkBase.ControlType.kPosition, pidSlot);
//    }
//
//    @Override
//    public CANSparkMAXRequest withSetPoint(Double setPoint) {
//        return
//    }
    public  int f(){}
    public  void f(){}

}
