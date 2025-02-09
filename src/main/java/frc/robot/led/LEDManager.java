package frc.robot.led;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class LEDManager {
    private static List<Pair<BooleanSupplier, LEDState>> suppliersList = new ArrayList<>();

    private LEDStateHandler ledStateHandler;

    public LEDManager(LEDStateHandler ledStateHandler) {
        this.ledStateHandler = ledStateHandler;
    }



    public void addCondition(Pair<BooleanSupplier, LEDState> ledStatePair){
        suppliersList.add(ledStatePair);
    }

    public void addCondition(BooleanSupplier supplier, LEDState state){
        addCondition(new Pair<>(supplier, state));
    }

    public void periodic(){
        setStateByBooleanSupplier();
    }

    public void setStateByBooleanSupplier(){
        for (Pair<BooleanSupplier, LEDState> p : suppliersList){
            if (p.getFirst().getAsBoolean()){
                ledStateHandler.setState(p.getSecond());
            }
        }
    }
}
