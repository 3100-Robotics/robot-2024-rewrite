// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Vision tagCamera = new Vision("april tags", Constants.Vision.kAprilTagCamTransform);
  // public final Vision tagCamera = null;
  public final Vision noteCamera = new Vision("note detector", new Transform3d());

  public final Drivetrain drive = new Drivetrain(noteCamera);
  public final Collector collector = new Collector();
  public final Pivot pivot = new Pivot();
  public final Shooter shooter = new Shooter();

  private final CommandXboxController driverController =
      new CommandXboxController(0);

//  private final CommandXboxController coDriverController =
//          new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putBoolean("is field oriented", true);

    drive.setDefaultCommand(drive.driveCommand(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            () -> SmartDashboard.getBoolean("is field oriented", false)));

//    shooter.setDefaultCommand(shooter.setCommand(0, 0));

    collector.setDefaultCommand(collector.runCommand(-0.3));

    // Configure the trigger bindings
    configureBindings();
  }

  private Command Collect() {
    return Commands.none();
  }

  public void configureAutonomous() {

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // a shoot
    // b amp
    // x auto collect
    // y manual collect
    // lb source collect
    // rb execute action

    // collecting
    driverController.leftBumper().onTrue(Commands.sequence(
       pivot.goToPos(Constants.pivotConstants.sourceCollectAngle),
       shooter.setCommand(-0.5, -0.3),
        Commands.waitUntil(shooter.noteInPosition()),
       shooter.setCommand(0, 0),
        pivot.goToPos(Constants.pivotConstants.collectAngle)));

      // TODO: reenable
    driverController.x().whileTrue(Commands.parallel(
            pivot.goToPos(Constants.pivotConstants.collectAngle),
            drive.autoCollect(shooter.noteInPosition()),
            shooter.setCommand(-0.6, -0.3),
            collector.runCommand(0.8)).
            andThen(Commands.waitUntil(shooter.noteInPosition())).
            andThen(shooter.setCommand(0, 0)));

//    driverController.y().onTrue(Commands.sequence(
//            pivot.goToPos(Constants.pivotConstants.collectAngle),
//            shooter.setCommand(-0.6, -0.3)
//                    .alongWith(collector.runCommand(0.8)),
//            Commands.waitUntil(shooter.noteInPosition()),
//            shooter.setCommand(0, 0)));

    // shooting
    // TODO: reenable
    driverController.b().onTrue(Commands.sequence(
            shooter.setVelInstantCommand(20, 0),
            pivot.goToPos(Constants.pivotConstants.shootAngle)));

    // trap
    // TODO: reenable
    driverController.y().onTrue(Commands.sequence(
            shooter.setVelInstantCommand(8000, 0),
            pivot.goToPos(Constants.pivotConstants.shootAngle+0.05)));


    // amp
    // TODO: reenable
    driverController.a().onTrue(Commands.sequence(
            shooter.setCommand(0.1, 0),
            pivot.goToPos(Constants.pivotConstants.ampAngle)));


    driverController.rightBumper().onTrue(Commands.sequence(
            shooter.setIndexerCommand(0.4),
            Commands.waitSeconds(0.75),
            shooter.setCommand(0, 0),
            pivot.goToPos(Constants.pivotConstants.collectAngle)));

    // driver commands

    // tuning

    //    driverController.a().whileTrue(pivot.goToPos(-0.178));
//        driverController.b().whileTrue(pivot.goToPos(0));
//        driverController.x().whileTrue(pivot.goToPos(0.126));
    //    driverController.y().whileTrue(pivot.goToPos(0.302));

//      driverController.a().whileTrue(shooter.setCommand(1, 0.4));
//      driverController.a().onFalse(shooter.setCommand(0, 0));
//      driverController.b().whileTrue(shooter.setCommand(0.2, 0.4));
//      driverController.b().onFalse(shooter.setCommand(0, 0));

    //    SignalLogger.start();
    //
//        driverController.a().whileTrue(shooter.sysidForwardDynamic());
//        driverController.b().whileTrue(shooter.sysidReverseDynamic());
//        driverController.x().whileTrue(shooter.sysidForwardStatic());
//        driverController.y().whileTrue(shooter.sysidReverseStatic());
    //
        driverController.povRight().onTrue(Commands.runOnce(SignalLogger::stop));
        driverController.povLeft().onTrue(Commands.runOnce(SignalLogger::start));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return drive.createPPTraj("test drive");
    return drive.createChoreoTraj("test drive");
    // andThen(Commands.waitSeconds(1)).
    // andThen(drive.createChoreoTraj("test drive.2"));
  }
}
