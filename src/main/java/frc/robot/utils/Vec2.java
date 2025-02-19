package frc.robot.utils;

public class Vec2 {
    private double x;
    private double y;

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double magnitude() {
        return Math.sqrt((x * x) + (y * y));
    }

    /**
     * Mutates this Vec2 into a unit vector. Returns the original magnitude.
     */
    public double normalize() {
        double mag = magnitude();
        if (mag >= 1e-6) {
            this.x /= mag;
            this.y /= mag;
        }
        return mag;
    }

    public void scale(double scale) {
        this.x *= scale;
        this.y *= scale;
    }

    public void update(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void update(Vec2 toCopy) {
        this.x = toCopy.x;
        this.y = toCopy.y;
    }
}
