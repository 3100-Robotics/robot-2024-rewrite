package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Shooter implements Subsystem {

    private TalonFX shooterMotor;
    private CANSparkMax indexerMotor;

    SysIdRoutine shooterSysId = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::setShooterVoltage, null, this));

    public Shooter() {
        shooterMotor = new TalonFX(Constants.shooterConstants.shooterMotorID);
        shooterMotor.getConfigurator().apply(Constants.shooterConstants.shooterConfigs);

        indexerMotor = new CANSparkMax(Constants.shooterConstants.indexerMotorID, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void setShooterVoltage(Measure<Voltage> volts) {
        shooterMotor.setVoltage(volts.baseUnitMagnitude());
    }
}

