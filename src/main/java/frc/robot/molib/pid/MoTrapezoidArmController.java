package frc.robot.molib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.molib.encoder.MoRotationEncoder;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class MoTrapezoidArmController
        extends MoTrapezoidController<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit> {
    private final Supplier<Angle> getAngleFromHorizontal;

    private double kS, kG, kV, kA;
    private ArmFeedforward armFF;

    public MoTrapezoidArmController(
            String name,
            SparkClosedLoopController pidController,
            ClosedLoopSlot pidSlot,
            MoRotationEncoder encoder,
            Consumer<Consumer<SparkBaseConfig>> configurator,
            Supplier<Angle> getAngleFromHorizontal) {
        super(
                name,
                pidController,
                pidSlot,
                encoder,
                configurator,
                Units.Rotations,
                Units.RotationsPerSecond,
                Units.RotationsPerSecondPerSecond);
        this.getAngleFromHorizontal = getAngleFromHorizontal;
    }

    public void setKS(double kS) {
        this.kS = kS;
        this.armFF = null;
    }

    public void setKG(double kG) {
        this.kG = kG;
        this.armFF = null;
    }

    public void setKV(double kV) {
        this.kV = kV;
        this.armFF = null;
    }

    public void setKA(double kA) {
        this.kA = kA;
        this.armFF = null;
    }

    @Override
    protected double getFF(Measure<AngularVelocityUnit> velSetpoint, Measure<AngularVelocityUnit> nextVelSetpoint) {
        if (this.armFF == null) {
            this.armFF = new ArmFeedforward(kS, kG, kV, kA);
        }
        return this.armFF.calculateWithVelocities(
                getAngleFromHorizontal.get().in(Units.Radians),
                velSetpoint.in(Units.RotationsPerSecond),
                nextVelSetpoint.in(Units.RotationsPerSecond));
    }
}
