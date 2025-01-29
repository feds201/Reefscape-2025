package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.DriveForwardCommand;
import frc.robot.commands.swerve.GameNavigator;
import frc.robot.constants.*;
import frc.robot.constants.RobotMap.SafetyMap;
import frc.robot.constants.RobotMap.SensorMap;
import frc.robot.constants.RobotMap.UsbMap;
import frc.robot.constants.RobotMap.SafetyMap.AutonConstraints;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.camera.Camera;
import frc.robot.utils.AutoPathFinder;
import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.ObjectType;
import frc.robot.utils.PoseAllocate;
import frc.robot.utils.RobotFramework;
import frc.robot.utils.SafetyManager;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;
import frc.robot.utils.Telemetry;

@SuppressWarnings("unused") // DO NOT REMOVE

public class RobotContainer extends RobotFramework {

    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private Telemetry telemetry;
    private final SendableChooser<Command> teleOpChooser;
    private final SendableChooser<Command> autonChooser;
    private final Camera frontCamera;
    private final Camera rearRightCamera;
    private final Camera rearLeftCamera;
    // private final Camera rearCamera;
    private final PathConstraints autoAlignConstraints;
    private final SwerveDrivePoseEstimator poseEstimator;

     private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("PoseEstimation");
    private final StructPublisher<Pose2d> estimatedPose = driveStateTable.getStructTopic("PoseEstimation", Pose2d.struct).publish();

    public RobotContainer() {
        double swerveSpeedMultiplier = 0.4;
        driverController = UsbMap.driverController;
        operatorController = UsbMap.operatorController;
        autoAlignConstraints = AutonConstraints.kPathConstraints;

        SwerveDriveState driveState = DrivetrainConstants.drivetrain.getState();
        Rotation2d gyroAngle = driveState.Pose.getRotation();
        SwerveModulePosition[] modulePositions = driveState.ModulePositions;
        poseEstimator = new SwerveDrivePoseEstimator(DrivetrainConstants.drivetrain.getKinematics(), gyroAngle, modulePositions, new Pose2d(0, 0, gyroAngle));

        swerveSubsystem = new SwerveSubsystem(
                Subsystems.SWERVE_DRIVE,
                Subsystems.SWERVE_DRIVE.getNetworkTable(),
                SensorMap.GYRO_PORT,
                driverController);

        frontCamera = new Camera(
                Subsystems.VISION,
                Subsystems.VISION.getNetworkTable(),
                ObjectType.APRIL_TAG_FRONT,
                "limelight-seven");

        rearRightCamera = new Camera(
                    Subsystems.VISION,
                    Subsystems.VISION.getNetworkTable(),
                    ObjectType.APRIL_TAG_BACK,
                    "limelight-one");

        rearLeftCamera = new Camera(
                        Subsystems.VISION,
                        Subsystems.VISION.getNetworkTable(),
                        ObjectType.APRIL_TAG_LEFT,
                        "limelight-six");

        // rearCamera = new Camera(
        //         Subsystems.VISION,
        //         Subsystems.VISION.getNetworkTable(),
        //         ObjectType.APRIL_TAG_BACK,
        //         "limelight-namegoeshere");

        teleOpChooser = new SendableChooser<>();
        setupDrivetrain();
        autonChooser = AutoBuilder.buildAutoChooser();
        DrivetrainConstants.drivetrain.setDefaultCommand(new Command() {

            {
                addRequirements(DrivetrainConstants.drivetrain, swerveSubsystem);
            }

            @Override
            public void execute() {
                Command selectedCommand = teleOpChooser.getSelected();
                if (selectedCommand != null) {
                    selectedCommand.schedule();
                }
            }
        });

        setupNamedCommands();
        setupPaths();
        configureBindings();


        telemetry = new Telemetry(SafetyMap.kMaxSpeed);
        DrivetrainConstants.drivetrain.registerTelemetry(telemetry::telemeterize);

    }

    public void setupVisionImplants() {
        var driveState = DrivetrainConstants.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        Rotation2d gyroAngle = driveState.Pose.getRotation();
        SmartDashboard.putNumber("robot rotation", headingDeg);
        double omega = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
        frontCamera.SetRobotOrientation(headingDeg, 0,0,0,0,0);
        rearRightCamera.SetRobotOrientation(headingDeg, 0,0,0,0,0);
        rearLeftCamera.SetRobotOrientation(headingDeg, 0,0,0,0,0);
        // rearCamera.SetRobotOrientation(headingDeg, 0,0,0,0,0);
        SwerveModulePosition[] modulePositions = driveState.ModulePositions;
        poseEstimator.update(gyroAngle, modulePositions);
        PoseAllocate frontPose = frontCamera.getRobotPose();
        PoseAllocate rearRightPose = rearRightCamera.getRobotPose();
        PoseAllocate rearLeftPose = rearLeftCamera.getRobotPose();
        // PoseAllocate rearPose = rearCamera.getRobotPose();

        // if  (
        //         frontPose != null
        //                 &&    frontPose.getPose() != null
        //                 && frontPose.getPoseEstimate().tagCount > 0
        //                 && Math.abs(omega) < 2) {
        //     DrivetrainConstants.drivetrain.addVisionMeasurement(frontPose.getPose(), frontPose.getTime());
            
        // }

        if  (
            rearLeftPose != null
                    &&    rearLeftPose.getPose() != null
                    && rearLeftPose.getPoseEstimate().tagCount > 0
                    && Math.abs(omega) < 2) {
        DrivetrainConstants.drivetrain.addVisionMeasurement(rearLeftPose.getPose(), rearLeftPose.getTime());
        
    }

    if  (
        rearRightPose != null
                &&    rearRightPose.getPose() != null
                && rearRightPose.getPoseEstimate().tagCount > 0
                && Math.abs(omega) < 2) {
    DrivetrainConstants.drivetrain.addVisionMeasurement(rearRightPose.getPose(), rearRightPose.getTime());
    
}
        // if  (
        //         rearPose != null
        //                 && rearPose.getPose() != null
        //                 && rearPose.getPoseEstimate().tagCount > 0
        //                 && Math.abs(omega) < 2) {
        //     DrivetrainConstants.drivetrain.addVisionMeasurement(rearPose.getPose(), rearPose.getTime());
        // }
        estimatedPose.set(poseEstimator.getEstimatedPosition());


    }

    private void configureBindings() {
        driverController.start()
                .onTrue(DrivetrainConstants.drivetrain
                        .runOnce(() -> DrivetrainConstants.drivetrain.seedFieldCentric()));

        // driverController.b()
        //         .onTrue(AutoPathFinder.GotoPath("Pathto1"));

        // driverController.y()
        //         .onTrue(AutoPathFinder.GotoPath("lineToRight"));

        driverController.leftBumper()
                .onTrue(GameNavigator.GoLeft(frontCamera.GetAprilTag()));

        driverController.rightBumper()
                .onTrue(GameNavigator.GoRight(frontCamera.GetAprilTag()));

        // driverController.rightBumper()
        //         .onTrue(GameNavigator.GoRight(frontCamera.getLastseenAprilTag()));

    }

    private void setupNamedCommands() {
        NamedCommands.registerCommand("Field Relative",
                DrivetrainConstants.drivetrain.runOnce(() -> DrivetrainConstants.drivetrain.seedFieldCentric()));
    }

    public void setupPaths() {
        autonChooser.setDefaultOption("Drive Forward", new DriveForwardCommand(swerveSubsystem, 0.5, 5));
        Shuffleboard.getTab(Subsystems.SWERVE_DRIVE.getNetworkTable()).add("Auton Chooser", autonChooser).withSize(2, 1)
                .withProperties(Map.of("position", "0, 0"));
    }

    public void setupDrivetrain() {
        teleOpChooser.setDefaultOption("Holo-Genic Drive", ConfigureHologenicDrive(driverController, swerveSubsystem));
        teleOpChooser.addOption("Arcade Drive", ConfigureArcadeDrive(driverController, swerveSubsystem));
        teleOpChooser.addOption("Tank Drive", ConfigureTankDrive(driverController, swerveSubsystem));
        teleOpChooser.addOption("Orbit Mode (Beta)", ConfigureOrbitMode(driverController, swerveSubsystem));
        teleOpChooser.addOption("BeyBlade (Maniac)", ConfigureBeyBlade(driverController, swerveSubsystem));
        teleOpChooser.addOption("FODC System (PID)", ConfigureFODC(driverController, swerveSubsystem));

        Shuffleboard.getTab(Subsystems.SWERVE_DRIVE.getNetworkTable()).add("TeleOp Chooser", teleOpChooser)
                .withSize(2, 1)
                .withProperties(Map.of("position", "0, 1"));
    }

    // DO NOT REMOVE
    public SubsystemABS[] SafeGuardSystems() {
        return new SubsystemABS[] {
                swerveSubsystem,
                frontCamera,
                rearLeftCamera,
                rearRightCamera

        };
    }

    // ONLY RUNS IN TEST MODE
    public Object[] TestCommands() {
        return new Object[] {
        };
    }

    public Object[] TestAutonCommands() {
        return new Object[] {
        };
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public Command getTeleOpCommand() {
        return teleOpChooser.getSelected();
    }

    public Command TestSystems() {
        return null;
    }

}
