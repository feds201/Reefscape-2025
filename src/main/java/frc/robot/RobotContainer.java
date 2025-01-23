package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;

import frc.robot.utils.*;
import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.util.Units;
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

@SuppressWarnings("unused") // DO NOT REMOVE

public class RobotContainer extends RobotFramework {

    private SwerveSubsystem swerveSubsystem;
    private CommandXboxController driverController;
    private CommandXboxController operatorController;
    private Telemetry telemetry;
    private SendableChooser<Command> teleOpChooser;
    private SendableChooser<Command> autonChooser;
    private Camera frontCamera;
    private Camera rearCamera;
    private PathConstraints autoAlignConstraints;

    public RobotContainer() {
        double swerveSpeedMultiplier = 0.4;
        driverController = UsbMap.driverController;
        operatorController = UsbMap.operatorController;
        autoAlignConstraints = AutonConstraints.kPathConstraints;

        swerveSubsystem = new SwerveSubsystem(
                Subsystems.SWERVE_DRIVE,
                Subsystems.SWERVE_DRIVE.getNetworkTable(),
                SensorMap.GYRO_PORT,
                driverController);

        // frontCamera = new Camera(
        //         Subsystems.VISION,
        //         Subsystems.VISION.getNetworkTable(),
        //         ObjectType.APRIL_TAG_FRONT);

        // rearCamera = new Camera(
        //         Subsystems.VISION,
        //         Subsystems.VISION.getNetworkTable(),
        //         ObjectType.APRIL_TAG_BACK);

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
        telemetry = new Telemetry(SafetyMap.kMaxSpeed);
        configureBindings();

        
        // setupVisionImplants();

    }

    // private void setupVisionImplants() {
    //     var driveState = DrivetrainConstants.drivetrain.getState();
    //     double headingDeg = driveState.Pose.getRotation().getDegrees();
    //     SmartDashboard.putNumber("robot rotation", headingDeg);
    //     double omega = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    //     frontCamera.SetRobotOrientation(headingDeg, 0,0,0,0,0);
    //     rearCamera.SetRobotOrientation(headingDeg, 0,0,0,0,0);

    //     PoseAllocate frontPose = frontCamera.getRobotPose();
    //     PoseAllocate rearPose = rearCamera.getRobotPose();

    //     if  (  
    //             frontPose != null 
    //         &&    frontPose.getPose() != null
    //         && frontPose.getPoseEstimate().tagCount > 0
    //         && Math.abs(omega) < 2) {
    //         DrivetrainConstants.drivetrain.addVisionMeasurement(frontPose.getPose(), frontPose.getTime());
    //     }
    //     if  (
    //                 rearPose != null
    //                     && rearPose.getPose() != null
    //                     && rearPose.getPoseEstimate().tagCount > 0
    //                     && Math.abs(omega) < 2) {
    //         DrivetrainConstants.drivetrain.addVisionMeasurement(rearPose.getPose(), rearPose.getTime());
    //     }



    // }

    private void configureBindings() {
        DrivetrainConstants.drivetrain.registerTelemetry(telemetry::telemeterize);
        driverController.start()
                .onTrue(DrivetrainConstants.drivetrain
                        .runOnce(() -> DrivetrainConstants.drivetrain.seedFieldCentric()));

        driverController.b()
                .onTrue(AutoPathFinder.GotoPath("Pathto1"));

        driverController.y()
                .onTrue(AutoPathFinder.GotoPath("lineToRight"));

        // driverController.leftBumper()
        //         .onTrue(GameNavigator.GoLeft(frontCamera.getLastseenAprilTag()));

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
                swerveSubsystem
                // ,
                // frontCamera,
                // rearCamera

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
