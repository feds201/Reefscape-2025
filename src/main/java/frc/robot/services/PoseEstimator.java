package frc.robot.services;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.DrivetrainConstants;

public class PoseEstimator {
    SwerveDrivePoseEstimator poseEstimator;
    Rotation2d gyroAngle;
    SwerveModulePosition[] modulePositions;
    // private final KalmanFilter<N3, N1, N3> kalmanFilter;

    public PoseEstimator(CommandSwerveDrivetrain drivetrain) {
        gyroAngle = DrivetrainConstants.drivetrain.getState().Pose.getRotation();
        modulePositions = DrivetrainConstants.drivetrain.getState().ModulePositions;
        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.drivetrain.getKinematics(),
            gyroAngle,
            modulePositions,
            new Pose2d(0, 0, gyroAngle),
            VecBuilder.fill(0.1, 0.1, 0.1), // State measurement standard deviations
            VecBuilder.fill(0.1, 0.1, 0.1)  // Vision measurement standard deviations
        );
        // LinearSystem<N3, N1, N3> drivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
            // 1.0,  // kv (velocity constant)
            // 0.5   // ka (acceleration constant)
        // );
        // kalmanFilter = new KalmanFilter<N3, N1, N3>(
        //     drivetrainPlant,
        //     VecBuilder.fill(0.1, 0.1, 0.1), // State measurement standard deviations
        //     VecBuilder.fill(0.1),           // Control input standard deviations
        //     0.02                            // Time step
        // );
    }

    public void update() {
        var driveState = DrivetrainConstants.drivetrain.getState();
        gyroAngle = driveState.Pose.getRotation();
        modulePositions = driveState.ModulePositions;

        // First update the built-in SwerveDrivePoseEstimator
        poseEstimator.update(gyroAngle, modulePositions);

        // Predict your Kalman filter with zero control input (placeholder), 20ms timestep
        // kalmanFilter.predict(VecBuilder.fill(0.0), 0.02);

        // Then correct with your measurement vector
        // kalmanFilter.correct(VecBuilder.fill(
        //     gyroAngle.getRadians(),
        //     modulePositions[0].distanceMeters,
        //     modulePositions[1].distanceMeters
        // ));
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
        // kalmanFilter.correct(VecBuilder.fill(visionPose.getX(), visionPose.getY(), visionPose.getRotation().getRadians()));
    }

    public Pose2d getEstimatedPosition() {
        return poseEstimator.getEstimatedPosition();
        // var state = kalmanFilter.getXhat();
        // return new Pose2d(state.get(0, 0), state.get(1, 0), new Rotation2d(state.get(2, 0)));
    }
}
