package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class vision {

    PhotonCamera camera;
    AprilTagFieldLayout tagLayout;

    public vision(String name) {
        camera = new PhotonCamera(name);
        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
}
