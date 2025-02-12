package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class PosePair {
    public int redAlliance;
    public int blueAlliance;
    public Pose2d leftpose;
    public Pose2d rightpose;


    public PosePair(int redAlliance, int blueAlliance, Pose2d leftpose, Pose2d rightpose) {
        this.redAlliance = redAlliance;
        this.blueAlliance = blueAlliance;
        this.leftpose = leftpose;
        this.rightpose = rightpose;
    }
    
    public Pose2d getLeftPath() {
        return leftpose;
    }

    public Pose2d getRightPath() {
        return rightpose;
    }

    public boolean tagToPath(int tag) {
        if (tag == redAlliance) {
            return true;
        } else if (tag == blueAlliance) {
            return true;
        } else {
            return false;
        }
    }

}