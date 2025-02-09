package frc.robot.contract;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.camera.Camera;

import java.util.List;

public class Graduated_Mean_Function {

    private Pose2d estimate = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    private double errorCovarianceX = 1.0;
    private double errorCovarianceY = 1.0;
    private double errorCovarianceTheta = 1.0;
    private final double processNoise = 1e-5;
    private final double measurementNoise = 1e-1;

    public Pose2d getBetterEstimate(List<Camera> cameras) {
        double totalWeight = 0.0;
        double weightedSumX = 0.0;
        double weightedSumY = 0.0;
        double weightedSumTheta = 0.0;

        // Calculate total weight and weighted sums
        for (Camera camera : cameras) {
            double weight = camera.getAprilTagCount();
            totalWeight += weight;
            weightedSumX += camera.getRobotPose().getPose().getX() * weight;
            weightedSumY += camera.getRobotPose().getPose().getY() * weight;
            weightedSumTheta += camera.getRobotPose().getPose().getRotation().getRadians() * weight;
        }

        // Calculate weighted means
        double weightedMeanX = weightedSumX / totalWeight;
        double weightedMeanY = weightedSumY / totalWeight;
        double weightedMeanTheta = weightedSumTheta / totalWeight;

        // Kalman filter update for x
        double kalmanGainX = errorCovarianceX / (errorCovarianceX + measurementNoise);
        double newX = estimate.getX() + kalmanGainX * (weightedMeanX - estimate.getX());
        errorCovarianceX = (1 - kalmanGainX) * errorCovarianceX + processNoise;

        // Kalman filter update for y
        double kalmanGainY = errorCovarianceY / (errorCovarianceY + measurementNoise);
        double newY = estimate.getY() + kalmanGainY * (weightedMeanY - estimate.getY());
        errorCovarianceY = (1 - kalmanGainY) * errorCovarianceY + processNoise;

        // Kalman filter update for theta
        double kalmanGainTheta = errorCovarianceTheta / (errorCovarianceTheta + measurementNoise);
        double newTheta = estimate.getRotation().getRadians() + kalmanGainTheta * (weightedMeanTheta - estimate.getRotation().getRadians());
        errorCovarianceTheta = (1 - kalmanGainTheta) * errorCovarianceTheta + processNoise;

        // Update estimate
        estimate = new Pose2d(newX, newY, new Rotation2d(newTheta));

        return estimate;
    }
}