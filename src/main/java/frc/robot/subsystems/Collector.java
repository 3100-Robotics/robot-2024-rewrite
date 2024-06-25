package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Collector implements Subsystem {

    private final CANSparkMax collectorMotor;

    public Collector() {
        collectorMotor = new CANSparkMax(
                Constants.collectorConstants.collectorMotorID,
                CANSparkLowLevel.MotorType.kBrushless);

        collectorMotor.setInverted(false);
        collectorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        collectorMotor.setSmartCurrentLimit(30);
    }

    public Command runCommand(double speed) {
        return this.runOnce(() -> collectorMotor.set(speed));
    }
}

