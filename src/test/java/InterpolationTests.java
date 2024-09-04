import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.Translation2dUtils;
import frc.utils.interpolator.DoubleInterpolator2D;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.Objects;

public class InterpolationTests {

    @Test
    public void closestAfterTest (){
        assert Objects.equals(Translation2dUtils.findClosestPointAfter(
                new Translation2d(0, 0),
                new Translation2d[]{
                        new Translation2d(1, 1),
                        new Translation2d(1, 2),
                        new Translation2d(2, 1),
                        new Translation2d(0, 1),
                        new Translation2d(-0.1,-0.1)
                }
        ), new Translation2d(0, 1));
    }
    @Test
    public void closestBeforeTest1 (){
        assert Objects.equals(Translation2dUtils.findClosestPointBefore(
                new Translation2d(0, 0),
                new Translation2d[]{
                        new Translation2d(1, 1),
                        new Translation2d(1, 2),
                        new Translation2d(2, 1),
                        new Translation2d(0, 1),
                }
        ), null);
    }
    @Test
    public void closestBeforeTest2 (){
        assert Objects.equals(Translation2dUtils.findClosestPointBefore(
                new Translation2d(0, 0),
                new Translation2d[]{
                        new Translation2d(1, 1),
                        new Translation2d(1, 2),
                        new Translation2d(2, 1),
                        new Translation2d(0, -1),
                }
        ), new Translation2d(0, -1));
    }

}
