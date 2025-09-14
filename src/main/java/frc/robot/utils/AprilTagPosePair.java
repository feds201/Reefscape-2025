package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class AprilTagPosePair {
    public Pose2d redAlliance;
    public Pose2d blueAlliance;
    public Pose2d leftpose;
    public Pose2d rightpose;
    public Pose2d centerPose;


   /** Class That matches Red and Blue Alliance Reef Tag Poses to Poses Along the Reef to Construct One Face of the Reef.
    * @param redAlliance Pose of Corresponding Red Alliance Reef Tag.
    * @param blueAlliance Pose of Corresponding Blue Alliance Reef Tag.
    * @param leftPose Pose of Robot When Properly Aligned to Corresponding Left Pole.
    * @param rightPose Pose of Robot When Properly Aligned to Corresponding Right Pole.
    * @param CenterPose Pose of Robot When Properly Aligned to Corresponding Center of Reef Face.
    */
    public AprilTagPosePair(Pose2d redAlliance, Pose2d blueAlliance, Pose2d leftpose, Pose2d rightpose, Pose2d centerPose) {
        this.redAlliance = redAlliance;
        this.blueAlliance = blueAlliance;
        this.leftpose = leftpose;
        this.rightpose = rightpose;
        this.centerPose = centerPose;
    }
    
    public Pose2d getLeftPose() {
        return leftpose;
    }

    public Pose2d getRightPose() {
        return rightpose;
    }

    public Pose2d getCenterPose(){
        return centerPose;
    }

    /** Method Used to Determine if a Target Face Corresponds with the Face the Object Represents.
     * @param tagPose Pose of the April Tag on the Target Reef Face.
     */
    public boolean poseToPath(Pose2d tagPose) {
        if (tagPose == redAlliance) {
            return true;
        } else if (tagPose == blueAlliance) {
            return true;
        } else {
            return false;
        }
    }

}