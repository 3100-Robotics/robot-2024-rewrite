package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Vision;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import com.choreo.lib.*;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Drive implements Subsystem {

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive drive;

   Vision tagCam;

    public Drive(Vision tagCam) {
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(46.42);
        // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
        //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
        //  The gear ratio is 6.75 motor revolutions per wheel rotation.
        //  The encoder resolution per motor revolution is 1 per motor revolution.
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(3), 
                Constants.driveConstants.driveGearRatio);



        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        try
        {
            drive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.driveConstants.maxSpeed);
            // Alternative method if you don't want to supply the conversion factor via JSON files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }
        drive.setHeadingCorrection(false);
        // Heading correction should only be used while controlling the robot via angle.
        drive.setCosineCompensator(false);
        // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.

       this.tagCam = tagCam;
    }

    @Override
    public void periodic() {
        drive.updateOdometry();
        SmartDashboard.putNumber("test number", drive.getMaximumAngularVelocity());
    }

    private void updateOdometry() {

    }

    public Command createTrajectory(String name) {
        ChoreoTrajectory traj = Choreo.getTrajectory(name); //

        Command resetPose = this.runOnce(() -> drive.resetOdometry(traj.getFlippedInitialPose()));

        return resetPose.andThen(Choreo.choreoSwerveCommand(
                traj,
                this::getPose,
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                drive::setChassisSpeeds,
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return  alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;},
                this));
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        drive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    private double filter(double input) {
        return Math.pow(MathUtil.applyDeadband(input, 0.03), 3);
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier heading,
                                BooleanSupplier isFieldOriented) {
        return this.run(() -> drive(new Translation2d(
                -filter(translationX.getAsDouble())*drive.getMaximumVelocity(),
                -filter(translationY.getAsDouble())*drive.getMaximumVelocity()),
                -filter(heading.getAsDouble())*drive.getMaximumAngularVelocity(),
                isFieldOriented.getAsBoolean()));
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        drive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void lock() {
        drive.lockPose();
    }
}

