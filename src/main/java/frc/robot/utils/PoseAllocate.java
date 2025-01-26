package frc.robot.utils;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;

public class PoseAllocate {
    private LimelightHelpers.PoseEstimate pose;
    private double time;

    public PoseAllocate(LimelightHelpers.PoseEstimate pose, double time) {
        this.pose = pose;
        this.time = time;
    }

    public Pose2d getPose() {
        return pose.pose;
    }

    public double getTime() {
        return Utils.fpgaToCurrentTime(time);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate() {
        return pose;
    }
}
