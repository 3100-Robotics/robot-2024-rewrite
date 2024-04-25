package frc.robot.subsystems;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;

public class Drive implements Subsystem {

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive drive;


    public Drive() {
        try {
            drive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
                    Constants.driveConstants.maxSpeed,
                    360,
                    SwerveMath.calculateMetersPerRotation(
                            Units.inchesToMeters(3),
                            Constants.driveConstants.driveGearRatio,
                            1));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    }
}

