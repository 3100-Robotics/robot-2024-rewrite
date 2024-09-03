package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.photonvision.PhotonCamera;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class outreachUtil {

    Consumer<ChassisSpeeds> driveController;
    Supplier<Pose2d> poseSupplier;
    PhotonCamera aprilTagCam;
    Transform3d aprilTagCamPos;
    PhotonCamera noteCam;
    Transform3d noteCamPos;
    AprilTagFieldLayout field;
    Subsystem requirements;
    double maxSpeed;

    public outreachUtil(
            Consumer<ChassisSpeeds> driveController,
            Supplier<Pose2d> poseSupplier,
            PhotonCamera aprilTagCam,
            Transform3d aprilTagCamPos,
            PhotonCamera noteCam,
            Transform3d noteCamPos,
            AprilTagFieldLayout field,
            double maxSpeed,
            Subsystem requirements) {


        this.driveController = driveController;
        this.poseSupplier = poseSupplier;
        this.aprilTagCam = aprilTagCam;
        this.noteCam = noteCam;
        this.aprilTagCamPos = aprilTagCamPos;
        this.noteCamPos = noteCamPos;
        this.field = field;
        this.requirements = requirements;
        this.maxSpeed = maxSpeed;
    }

    public Command defaultDriveCommand(DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier rotation) {
        return Commands.run(() -> {
            ChassisSpeeds speeds = new ChassisSpeeds(
                    driveX.getAsDouble()*maxSpeed,
                    driveY.getAsDouble()*maxSpeed,
                    rotation.getAsDouble());
            Pose2d pose = poseSupplier.get();
            if (pose.getX() < 0.5 || pose.getX() > (field.getFieldLength() - 0.5)) {
                speeds.vxMetersPerSecond *= 0.25;
            }
            if (pose.getY() < 0.5 || pose.getY() > (field.getFieldWidth() - 0.5)) {
                speeds.vyMetersPerSecond *= 0.25;
            }
            
            }
            , requirements);
    }
}
