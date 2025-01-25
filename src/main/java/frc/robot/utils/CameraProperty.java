package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;

public class CameraProperty {
    private String cameraName;
    private Pose3d cameraPose;

    public CameraProperty(String cameraName, Pose3d cameraPose) {
        this.cameraName = cameraName;
        this.cameraPose = cameraPose;
    }

    public String getCameraName() {
        return cameraName;
    }

    public Pose3d getCameraPose() {
        return cameraPose;
    }

    public CameraProperty(String cameraName) {
        this.cameraName = cameraName;
        this.cameraPose = new Pose3d();
    }
    
}
