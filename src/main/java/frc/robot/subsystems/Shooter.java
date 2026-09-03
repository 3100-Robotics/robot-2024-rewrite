package frc.robot.subsystems;


import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Volts;

public class Shooter implements Subsystem {

    private TalonFX shooterMotor;
    private SparkMax indexerMotor;

    private LaserCan noteSensor;

    private Trigger noteSensorActive;

    private SysIdRoutine SYSID = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,         // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                    null,          // Use default timeout (10 s)
                    // Log state with Phoenix SignalLogger class
                    (state)-> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(this::setShooterVoltage, null, this));

    public Shooter() {
        shooterMotor = new TalonFX(Constants.shooterConstants.shooterMotorID);
        shooterMotor.getConfigurator().apply(Constants.shooterConstants.shooterConfigs);

        indexerMotor = new SparkMax(Constants.shooterConstants.indexerMotorID, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig imotorconf = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(20);
        indexerMotor.configure(imotorconf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        noteSensor = new LaserCan(Constants.shooterConstants.laserCanID);
        try {
            noteSensor.setRangingMode(LaserCan.RangingMode.SHORT);
        } catch (ConfigurationFailedException ignored) {
            // throw new RuntimeException("No lasercan!");
        }

        noteSensorActive = new Trigger(() -> noteSensor.getMeasurement().distance_mm < 100);
    }

    public Command setCommand(double shooterSpeed, double indexerSpeed) {
        return this.runOnce(() -> {
            shooterMotor.set(shooterSpeed);
            indexerMotor.set(indexerSpeed);});
    }

    /**
     * only sets the velocity of the shooter indexer is still a %
     */
    public Command setVelCommand(double shooterVel, double indexerSpeed) {
        return this.run(() -> {
            shooterMotor.setControl(new MotionMagicVelocityVoltage(shooterVel));
            indexerMotor.set(indexerSpeed);
        });
    }

    public Command setVelInstantCommand(double shooterVel, double indexerSpeed) {
        return this.runOnce(() -> {
            shooterMotor.setControl(new MotionMagicVelocityVoltage(shooterVel));
            indexerMotor.set(indexerSpeed);
        });
    }

    public Trigger noteInPosition() {
        return noteSensorActive;
    }

    public void setShooterVoltage(Voltage volts) {
        shooterMotor.setVoltage(volts.baseUnitMagnitude());
    }

    public Command sysidForwardStatic() {
        return SYSID.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysidReverseStatic() {
        return SYSID.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysidForwardDynamic() {
        return SYSID.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysidReverseDynamic() {
        return SYSID.dynamic(SysIdRoutine.Direction.kReverse);
    }
}

