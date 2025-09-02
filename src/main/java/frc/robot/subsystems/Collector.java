package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Collector implements Subsystem {

    private final SparkMax collectorMotor;

    public Collector() {
        collectorMotor = new SparkMax(
                Constants.collectorConstants.collectorMotorID,
                SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig cmotorconfig = new SparkMaxConfig()
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        // collectorMotor.setInverted(false);
        // collectorMotor.setIdleMode(SparkBase.IdleMode.kBrake);
        // collectorMotor.setSmartCurrentLimit(30);

        collectorMotor.configure(cmotorconfig, 
            ResetMode.kNoResetSafeParameters, 
            PersistMode.kNoPersistParameters);
    }

    public Command runCommand(double speed) {
        return this.runOnce(() -> collectorMotor.set(speed));
    }
}

