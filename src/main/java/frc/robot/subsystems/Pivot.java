package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.pivotConstants;

public class Pivot implements Subsystem {
    
    private final TalonFX pivotMotor;

    private CANcoder pivotEncoder;

    private double setpoint;

    private final Trigger atSetpoint;

    private SysIdRoutine SYSID = new SysIdRoutine(
        new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                null,          // Use default timeout (10 s)
                                       // Log state with Phoenix SignalLogger class
                (state)->SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(this::setVoltage, null, this));

    public Pivot() {
        pivotMotor = new TalonFX(pivotConstants.pivotMotorID);
        pivotMotor.getConfigurator().apply(pivotConstants.pivotMotorConfigs);

        pivotEncoder = new CANcoder(pivotConstants.pivotEncoderID);
        pivotEncoder.getConfigurator().apply(pivotConstants.pivotEncoderConfigs);

        atSetpoint = new Trigger(() -> Math.abs(pivotMotor.getPosition().getValueAsDouble() - setpoint) < 0.005);
    }

    @Override
    public void periodic() {
//        SmartDashboard.putNumber("pivot pos", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("at pivot setpoint", atSetpoint.getAsBoolean());
    }

    public double getPivotPos() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public Command goToPos(double pos) {
        return this.runOnce(() -> {
            setpoint = pos;
            pivotMotor.setControl(new MotionMagicVoltage(pos));})
                .andThen(Commands.waitUntil(atSetpoint));
    }

    public Trigger atSetpoint() {
        return atSetpoint;
    }

    public void setVoltage(Measure<Voltage> volts) {
        pivotMotor.setVoltage(volts.baseUnitMagnitude());
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
