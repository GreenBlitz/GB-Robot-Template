import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.Translation2dUtils;
import frc.utils.interpolator.DoubleInterpolator2D;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Objects;

public class InterpolationTests {

    private final DoubleInterpolator2D interpolator = new DoubleInterpolator2D();
    @BeforeEach
    public void restart(){
        interpolator.empty();

        interpolator.put(
                new Translation2d(0,0), 0
        );

        interpolator.put(
                new Translation2d(1,1), 1
        );
        interpolator.put(
                new Translation2d(0.5,0.5),2
        );
    }

    @Test
    public void test1(){
        assert interpolator.get(new Translation2d(0.25,0.25)) == 1;
    }
    @Test
    public void test2(){
        assert interpolator.get(new Translation2d(0.5,0.5)) == 2;
    }
    @Test
    public void test3(){
        assert interpolator.get(new Translation2d(0.25,0.25)) == 1;
    }
    @Test
    public void test4(){
        assert interpolator.get(new Translation2d(0.75,0.75)) == 1.5;
    }

}
