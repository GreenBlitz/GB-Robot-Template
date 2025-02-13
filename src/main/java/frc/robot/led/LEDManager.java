package frc.robot.led;

import edu.wpi.first.math.Pair;
import frc.robot.Robot;
import frc.robot.statemachine.superstructure.SuperstructureState;
import frc.utils.DriverStationUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class LEDManager {
    private static List<Pair<BooleanSupplier, LEDState>> suppliersList = new ArrayList<>();

    private LEDStateHandler ledStateHandler;
    
    private Robot robot;

    public LEDManager(Robot robot, LEDStateHandler ledStateHandler) {
        this.ledStateHandler = ledStateHandler;
        this.robot = robot;
    }



    public void addCondition(Pair<BooleanSupplier, LEDState> ledStatePair){
        suppliersList.add(ledStatePair);
    }

    public void addCondition(BooleanSupplier supplier, LEDState state){
        addCondition(new Pair<>(supplier, state));
    }

    public void periodic(){
        setAllStateByBooleanSupplier();
    }

    public void setAllStateByBooleanSupplier(){
        for (Pair<BooleanSupplier, LEDState> ledStatePair : suppliersList){
            setStateByBooleanSupplier(ledStatePair);
        }
    }

    public void setStateByBooleanSupplier(Pair<BooleanSupplier, LEDState> ledStatePair){
        if (ledStatePair.getFirst().getAsBoolean()){
            ledStateHandler.setState(ledStatePair.getSecond()).schedule();
        }
    }
    
    public void initializeLEDManager(){
        addCondition(
                () -> robot.getRobotCommander().getSuperstructure().getCurrentState() == SuperstructureState.IDLE,
                LEDState.IDLE
        );
        
        addCondition(
                () -> DriverStationUtil.isDisabled(),
                LEDState.DISABLE
        );
        
        addCondition(
                () -> robot.getRobotCommander().getSuperstructure().isCoralIn(),
                LEDState.HAS_CORAL
        );
        
        addCondition(
                () -> robot.getRobotCommander().getSuperstructure().isReadyToScore(),
                LEDState.IN_POSITION_TO_SCORE
        );
        
        
    }
}
