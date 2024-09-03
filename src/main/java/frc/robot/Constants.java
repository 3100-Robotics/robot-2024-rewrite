// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class driveConstants {
    public static final double maxSpeed = Units.feetToMeters(14);
    public static final double driveGearRatio = 4.71;

    public static final double autoCollectP = 0.05;
    public static final double autoCollectI = 0;
    public static final double autoCollectD = 0;
    public static final double autoCollectForwardSpeed = 1;
    public static final double autoCollectMaxSideSpeed = 0.5;

    public static final double autoShootP = 1;
    public static final double autoShootI = 0;
    public static final double autoShootD = 0;
  }

  public static class shooterConstants {
    public static final int shooterMotorID = 9;
    public static final int indexerMotorID = 10;
    public static final int laserCanID = 11;

    public static TalonFXConfiguration shooterConfigs = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withAudio(new AudioConfigs()
                    .withBeepOnConfig(true)
                    .withBeepOnBoot(true)
                    .withAllowMusicDurDisable(true))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true))
            .withSlot0(new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKP(0.5)
                    .withKI(0)
                    .withKD(0)
                    .withKG(0)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0))
            .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(1))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(100));
  }

  public static class pivotConstants {
    public static final int pivotMotorID = 12;
    public static final int pivotEncoderID = 13;

    public static TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitThreshold(0)
                    .withReverseSoftLimitThreshold(0)
                    .withForwardSoftLimitEnable(false)
                    .withReverseSoftLimitEnable(false))
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLimitEnable(true))
            .withAudio(new AudioConfigs()
                    .withBeepOnBoot(true)
                    .withBeepOnConfig(true)
                    .withAllowMusicDurDisable(true))
            .withFeedback(new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    .withFeedbackRemoteSensorID(pivotEncoderID)
                    .withRotorToSensorRatio(61.5385)
                    .withSensorToMechanismRatio(1))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(1)
                    .withMotionMagicCruiseVelocity(3))
            .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(1))
            .withSlot0(new Slot0Configs()
                    .withKP(61.047)
                    .withKI(0)
                    .withKD(5)//11.137
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKG(0.075898)
                    .withKS(0.19168)
                    .withKA(0.93568)
                    .withKA(0.93568));

    public static CANcoderConfiguration pivotEncoderConfigs = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                    .withMagnetOffset(-0.7822265625)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    public static final double collectAngle = -0.19;
    public static final double sourceCollectAngle = 0.3010;
    public static final double ampAngle = 0.3210;
    public static final double shootAngle = 0.1340;
  }

  public static class collectorConstants {
    public static final int collectorMotorID = 14;
  }

  public static class Vision {
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kAprilTagCamTransform =
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
