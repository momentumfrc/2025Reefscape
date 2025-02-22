package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import java.util.function.Supplier;

/**
 * A reimplementation of the built-in wpilib {@link edu.wpi.first.math.filter.SlewRateLimiter SlewRateLimiter} that
 * allows for on-the-fly updates to the slew rate limit.
 */
public class VariableSlewRateLimiter {
    private final Supplier<Double> limitSupplier;
    private double prevVal;
    private double prevTime;

    public VariableSlewRateLimiter(Supplier<Double> limitSupplier) {
        this.limitSupplier = limitSupplier;
        prevVal = 0;
        prevTime = MathSharedStore.getTimestamp();
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - prevTime;
        double rateLimit = limitSupplier.get();
        prevVal += MathUtil.clamp(input - prevVal, -rateLimit * elapsedTime, rateLimit * elapsedTime);
        prevTime = currentTime;
        return prevVal;
    }
}
