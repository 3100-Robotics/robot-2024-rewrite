package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import com.choreo.lib.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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

    double rateLimit = 8;
    double maxSpeed = 2;
    double maxRotation = 4;

    SlewRateLimiter xLimiter = new SlewRateLimiter(rateLimit, -5, 0);
    SlewRateLimiter yLimiter = new SlewRateLimiter(rateLimit, -5, 0);

    public Drivetrain(Vision noteCam) {
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
        // drive.pushOffsetsToControllers();
        drive.pushOffsetsToEncoders();
        drive.setHeadingCorrection(false);

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

       setupPathPlanner();
    }

    @Override
    public void periodic() {
        drive.updateOdometry();
        // updateOdometry();
        SmartDashboard.putNumber("test number", drive.getMaximumAngularVelocity());
        SmartDashboard.putNumber("pos x", drive.getPose().getX());
        SmartDashboard.putNumber("pos y", drive.getPose().getY());
    }

    private void updateOdometry() {
        Optional<EstimatedRobotPose> pose = tagCam.getEstimatedGlobalPose();

        pose.ifPresent(estimatedRobotPose ->
                drive.addVisionMeasurement(
                        estimatedRobotPose.estimatedPose.toPose2d(),
                        estimatedRobotPose.timestampSeconds));
    }

    public void setupPathPlanner()  {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            drive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            drive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                            new PIDConstants(1),
                                            // Translation PID constants
                                            new PIDConstants(0.4, 0, 0.01),
                                            // Rotation PID constants
                                            2,
                                            // Max module speed, in m/s
                                            drive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                            // Drive base radius in meters. Distance from robot center to furthest module.
                                            new ReplanningConfig()
                                            // Default path replanning config. See the API for the options here
            ),
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this);
    }

    public Command createPPTraj(String pathName)  {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return new PathPlannerAuto(pathName);
  }

    public Command createChoreoTraj(String name) {
        ChoreoTrajectory traj = Choreo.getTrajectory(name); //

        BooleanSupplier needToFlip = () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return  alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;};

        Command resetPose;
        if (needToFlip.getAsBoolean()) {
            resetPose = this.runOnce(() -> drive.resetOdometry(traj.getFlippedInitialPose()));
        }
        else {
            resetPose = this.runOnce(() -> drive.resetOdometry(traj.getInitialPose()));
        }

        

        return resetPose.andThen(Choreo.choreoSwerveCommand(
                traj,
                this::getPose,
                new PIDController(1.5, 0.0, 0.0),
                new PIDController(1.5, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0),
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

    private double rateLimit(double input, SlewRateLimiter limiter) {
        return Math.copySign(limiter.calculate(Math.abs(input)), input);
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier heading,
                                BooleanSupplier isFieldOriented) {
        return this.run(() -> drive(new Translation2d(
                -rateLimit(filter(translationX.getAsDouble())*maxSpeed, xLimiter),
                -rateLimit(filter(translationY.getAsDouble())*maxSpeed, yLimiter)),
                -filter(heading.getAsDouble())*maxRotation,
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

