// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.momentum4999.motune.PIDTuner;
import com.momentum4999.motune.PIDTunerBuilder;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import frc.robot.Constants;
import frc.robot.molib.pid.MoSparkMaxArmPID;
import frc.robot.molib.pid.MoSparkMaxElevatorPID;
import frc.robot.molib.pid.MoSparkMaxPID;
import frc.robot.molib.pid.MoTalonFxPID;

public class TunerUtils {

    private static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> PIDTunerBuilder moSparkBase(
            MoSparkMaxPID<Dim, VDim> sparkMax, String controllerName) {
        PIDTunerBuilder builder = PIDTuner.builder(controllerName)
                .withDataStoreFile(Constants.DATA_STORE_FILE)
                .withP(sparkMax::setP)
                .withI(sparkMax::setI)
                .withD(sparkMax::setD)
                .withIZone(sparkMax::setIZone)
                .withSetpoint(sparkMax::getSetpoint)
                .withMeasurement(sparkMax::getLastMeasurement);

        if (sparkMax.getType() == MoSparkMaxPID.Type.SMARTMOTION) {
            builder = builder.withProperty(
                            "maxVel",
                            (v) -> sparkMax.setConfigOption(
                                    c -> c.closedLoop.maxMotion.maxVelocity(v, sparkMax.getPidSlot())))
                    .withProperty(
                            "maxAccel",
                            (a) -> sparkMax.setConfigOption(
                                    c -> c.closedLoop.maxMotion.maxAcceleration(a, sparkMax.getPidSlot())))
                    .withProperty(
                            "allowedError",
                            (e) -> sparkMax.setConfigOption(c -> c.closedLoop.maxMotion.allowedClosedLoopError(e)));
        } else if (sparkMax.getType() == MoSparkMaxPID.Type.SMARTVELOCITY) {
            builder = builder.withProperty(
                            "maxAccel",
                            (a) -> sparkMax.setConfigOption(
                                    c -> c.closedLoop.maxMotion.maxAcceleration(a, sparkMax.getPidSlot())))
                    .withProperty(
                            "allowedError",
                            (e) -> sparkMax.setConfigOption(c -> c.closedLoop.maxMotion.allowedClosedLoopError(e)));
        }

        return builder;
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> PIDTuner forMoSparkMax(
            MoSparkMaxPID<Dim, VDim> sparkMax, String controllerName) {
        return moSparkBase(sparkMax, controllerName).withFF(sparkMax::setFF).safeBuild();
    }

    public static PIDTuner forMoSparkArm(MoSparkMaxArmPID armPID, String controllerName) {
        return moSparkBase(armPID, controllerName)
                .withProperty("ff_builtin", armPID::setFF)
                .withProperty("ff_kS", armPID::setKS)
                .withProperty("ff_kG", armPID::setKG)
                .withProperty("ff_kV", armPID::setKV)
                .withStateValue("calculated_ff", armPID::getLastFF)
                .safeBuild();
    }

    public static PIDTuner forMoSparkElevator(MoSparkMaxElevatorPID controller, String controllerName) {
        return moSparkBase(controller, controllerName)
                .withProperty("ff_builtin", controller::setFF)
                .withProperty("ff_kS", controller::setKS)
                .withProperty("ff_kG", controller::setKG)
                .withProperty("ff_kV", controller::setKV)
                .withStateValue("calculated_ff", controller::getLastFF)
                .safeBuild();
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> PIDTuner forMoTalonFx(
            MoTalonFxPID<Dim, VDim> talon, String controllerName) {
        return PIDTuner.builder(controllerName)
                .withDataStoreFile(Constants.DATA_STORE_FILE)
                .withP(talon::setP)
                .withI(talon::setI)
                .withD(talon::setD)
                .withFF(talon::setFF)
                .withIZone(talon::setIZone)
                .withSetpoint(talon::getSetpoint)
                .withMeasurement(talon::getLastMeasurement)
                .safeBuild();
    }
}
