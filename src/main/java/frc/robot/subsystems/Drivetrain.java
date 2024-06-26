package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Vision;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
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

public class Drivetrain implements Subsystem {

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive drive;

   Vision tagCam, noteCam;

   PIDController autoCollectingPID;
   PIDController autoAimingPID;

    public Drivetrain(Vision tagCam, Vision noteCam) {
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
        drive.setCosineCompensator(false);
        drive.pushOffsetsToControllers();
        drive.setHeadingCorrection(false);

       this.tagCam = tagCam;
       this.noteCam = noteCam;

       autoCollectingPID = new PIDController(
               Constants.driveConstants.autoCollectP,
               Constants.driveConstants.autoCollectI,
               Constants.driveConstants.autoCollectD);
       autoCollectingPID.setSetpoint(0);

       autoAimingPID = new PIDController(
               Constants.driveConstants.autoShootP,
               Constants.driveConstants.autoShootI,
               Constants.driveConstants.autoShootD);
       autoAimingPID.setSetpoint(0);
    }

    @Override
    public void periodic() {
        drive.updateOdometry();
        updateOdometry();
        SmartDashboard.putNumber("test number", drive.getMaximumAngularVelocity());
    }

    private void updateOdometry() {
        Optional<EstimatedRobotPose> pose = tagCam.getEstimatedGlobalPose();

        pose.ifPresent(estimatedRobotPose ->
                drive.addVisionMeasurement(
                        estimatedRobotPose.estimatedPose.toPose2d(),
                        estimatedRobotPose.timestampSeconds));
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

    public Command autoCollect(Trigger runUntil) {
        return this.run(() -> {
            PhotonPipelineResult results = noteCam.getLatestResult();
            if (results.hasTargets()) {
                double speed = autoCollectingPID.calculate(results.getBestTarget().getYaw());
                drive(new Translation2d(
                        Constants.driveConstants.autoCollectForwardSpeed,
                        Math.min(speed, Constants.driveConstants.autoCollectMaxSideSpeed)),
                        0, false);
            }
            else {
                drive(new Translation2d(), 0, false);
            }
        }).until(runUntil);
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

