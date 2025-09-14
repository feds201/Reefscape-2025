package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
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
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auton.FeedThenL3;
import frc.robot.commands.auton.MoveForward;
import frc.robot.commands.auton.MoveRobotCentricX;
import frc.robot.commands.auton.posePathfindToReef;
import frc.robot.commands.climber.RaiseClimberBasic;
import frc.robot.commands.climber.ServoOut;
import frc.robot.commands.climber.climbingSequenceUp;
import frc.robot.commands.lift.HoldElevator;
import frc.robot.commands.lift.RotateElevatorBasic;
import frc.robot.commands.lift.RotateElevatorDownPID;
import frc.robot.commands.lift.RotateElevatorNextLevelDown;
import frc.robot.commands.lift.RotateElevatorNextLevelUp;
import frc.robot.commands.lift.RotateElevatorPID;
import frc.robot.commands.lift.RotateElevatorSafePID;
import frc.robot.commands.swanNeck.PlaceLTwo;
import frc.robot.commands.swanNeck.RaiseSwanNeck;
import frc.robot.commands.swanNeck.RaiseSwanNeckPID;
import frc.robot.commands.swanNeck.RaiseSwanNeckPIDAlgae;
import frc.robot.commands.swanNeck.RaiseSwanNeckToScoringAngle;
import frc.robot.commands.swanNeck.PlaceLThree;
import frc.robot.commands.swanNeck.SpinSwanWheels;
import frc.robot.commands.swanNeck.retriveAlgae;
import frc.robot.commands.swanNeck.IntakeAlgaeFromGround;
import frc.robot.commands.swanNeck.IntakeCoralSequence;
import frc.robot.commands.swanNeck.PlaceBarge;
import frc.robot.commands.swanNeck.PlaceLFour;
import frc.robot.commands.swanNeck.PlaceLFourAuton;
import frc.robot.commands.swanNeck.PlaceLOne;
import frc.robot.commands.swerve.ConfigureHologenicDrive;
import frc.robot.commands.swerve.ConfigureSlowDrive;
import frc.robot.commands.swerve.DriveForwardCommand;
import frc.robot.commands.vision.RetrieveClosestGamePiece;
import frc.robot.constants.*;
import frc.robot.constants.RobotMap.ElevatorMap;
import frc.robot.constants.RobotMap.IntakeMap;
import frc.robot.constants.RobotMap.SafetyMap;
import frc.robot.constants.RobotMap.SensorMap;
import frc.robot.constants.RobotMap.SwerveMap;
import frc.robot.constants.RobotMap.UsbMap;
import frc.robot.constants.RobotMap.IntakeMap.ReefStops;
import frc.robot.constants.RobotMap.SafetyMap.AutonConstraints;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;
import frc.robot.subsystems.swanNeck.SwanNeckWheels;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.camera.Camera;
import frc.robot.utils.AutoPathFinder;
import frc.robot.utils.DrivetrainConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ObjectType;
import frc.robot.utils.PoseAllocate;
import frc.robot.utils.PoseEstimator;
import frc.robot.utils.RobotFramework;
import frc.robot.utils.SafetyManager;
import frc.robot.utils.SubsystemABS;
import frc.robot.utils.Subsystems;
import frc.robot.utils.Telemetry;

@SuppressWarnings("unused") // DO NOT REMOVE

public class RobotContainer extends RobotFramework {

    private final SwerveSubsystem swerveSubsystem;
    public final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final CommandXboxController testController;
    private Telemetry telemetry;
    private final SendableChooser<Command> teleOpChooser;
    private final SendableChooser<Command> autonChooser;
    private SendableChooser<Command> commandChooser;
    private final Camera frontRightCamera;
    public Camera frontLeftCamera;
    private final Camera rearRightCamera;
    // private final Camera rearLeftCamera;
    private final Climber climber;
    public Command zeroMechanisms;

    // private final Camera rearCamera;
    private final PathConstraints autoAlignConstraints;
    private final PoseEstimator poseEstimator;
    private Lift elevator;
    private SwanNeck swanNeck;
    private SwanNeckWheels swanNeckWheels;
    private PathPlannerPath leftAutoPath;
    private PathPlannerPath centerAutoPath;
    private PathPlannerPath jackAutoPath;

    //code project - instantiate pathplanner auto section commands here (like how you see below)

    private Command Left3L4Part2;
    private Command Left3L4Part3;
    private Command Left3L4Part1;




    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    public RobotContainer() {
        double swerveSpeedMultiplier = 0.4;
        driverController = UsbMap.driverController;
        operatorController = UsbMap.operatorController;
        autoAlignConstraints = AutonConstraints.kPathConstraints;
        testController = new CommandXboxController(2);

        poseEstimator = new PoseEstimator(DrivetrainConstants.drivetrain);

        swanNeckWheels = new SwanNeckWheels(
                Subsystems.SWAN_NECK_ROLLER,
                Subsystems.SWAN_NECK_ROLLER.getNetworkTable());
        climber = new Climber(
                Subsystems.CLIMBER,
                Subsystems.CLIMBER.getNetworkTable());

        elevator = new Lift(
                Subsystems.ELEVATOR,
                Subsystems.ELEVATOR.getNetworkTable());

        swerveSubsystem = new SwerveSubsystem(
                Subsystems.SWERVE_DRIVE,
                Subsystems.SWERVE_DRIVE.getNetworkTable(),
                SensorMap.GYRO_PORT,
                driverController);

        frontRightCamera = new Camera(
                Subsystems.VISION,
                Subsystems.VISION.getNetworkTable(),
                ObjectType.APRIL_TAG_FRONT,
                "limelight-seven");

        frontLeftCamera = new Camera(
                Subsystems.VISION, Subsystems.VISION.getNetworkTable(), ObjectType.APRIL_TAG_FRONT_LEFT,
                "limelight-five");

        rearRightCamera = new Camera(Subsystems.VISION, Subsystems.VISION.getNetworkTable(), ObjectType.APRIL_TAG_BACK,
                "limelight-three");

        // rearLeftCamera = new Camera(Subsystems.VISION,
        // Subsystems.VISION.getNetworkTable(), ObjectType.APRIL_TAG_LEFT,
        // "limelight-one");

        swanNeck = new SwanNeck(
                Subsystems.INTAKE,
                Subsystems.INTAKE.getNetworkTable());
        telemetry = new Telemetry(5);

        teleOpChooser = new SendableChooser<>();

        setupDrivetrain();

        DrivetrainConstants.drivetrain
                .setDefaultCommand(DrivetrainConstants.drivetrain.applyRequest(() -> DrivetrainConstants.drive
                        .withVelocityX(-driverController.getLeftY() * SafetyMap.kMaxSpeed
                                * SafetyMap.kMaxSpeedChange)
                        .withVelocityY(-driverController.getLeftX() * SafetyMap.kMaxSpeed
                                * SafetyMap.kMaxSpeedChange)
                        .withRotationalRate(-driverController.getRightX()
                                * SafetyMap.kMaxAngularRate
                                * SafetyMap.kAngularRateMultiplier)));

        zeroMechanisms = new InstantCommand(() -> elevator.zeroElevator())
                .alongWith(new InstantCommand(() -> swanNeck.zeroPivotPosition()))
                .alongWith(new InstantCommand(() -> climber.zeroClimber()));

        setupEventTriggers();
        setupNamedCommands();

       //code project - setup right auton auto sections here (like how you see below)

        Left3L4Part2 = AutoBuilder.buildAuto("Left3L4Part2");
        Left3L4Part3 = AutoBuilder.buildAuto("Left3L4Part3");
        Left3L4Part1 = AutoBuilder.buildAuto("Left3L4Part1");
        



        boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autonChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
          (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
        );
        setupPaths();
        SmartDashboard.putData(autonChooser);
        leftAutoPath = AutoPathFinder.loadPath("LeftAutoSetup");
        centerAutoPath = AutoPathFinder.loadPath("CenterAutoSetup");
        jackAutoPath = AutoPathFinder.loadPath("JackAutoSetup");
        configureBindings();

        telemetry = new Telemetry(SafetyMap.kMaxSpeed);
        DrivetrainConstants.drivetrain.registerTelemetry(telemetry::telemeterize);

    }

    // ADD: Getter for Elevator
    public Lift getElevator() {
        return elevator;
    }

    public void setupVisionImplantsTele() {
        var driveState = DrivetrainConstants.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        SmartDashboard.putNumber("heading Deg", headingDeg);
        double omega = Math.abs(Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond));
        frontRightCamera.SetRobotOrientation(headingDeg, 0, 0, 0, 0, 0);
        frontLeftCamera.SetRobotOrientation(headingDeg, 0, 0, 0, 0, 0);
        rearRightCamera.SetRobotOrientation(headingDeg, 0, 0, 0, 0, 0);

        PoseAllocate backRightPose = rearRightCamera.getRobotPose();
        PoseAllocate frontRightPose = frontRightCamera.getRobotPose();
        PoseAllocate frontLeftPose = frontLeftCamera.getRobotPose();

        if (frontRightPose != null
                && frontRightPose.getPose() != null
                && frontRightPose.getPoseEstimate().tagCount > 0
                && omega < 2) {
            DrivetrainConstants.drivetrain.addVisionMeasurement(frontRightPose.getPose(), frontRightPose.getTime());

        }

        if (frontLeftPose != null
                && frontLeftPose.getPose() != null
                && frontLeftPose.getPoseEstimate().tagCount > 0
                && omega < 2) {
            DrivetrainConstants.drivetrain.addVisionMeasurement(frontLeftPose.getPose(), frontLeftPose.getTime());

        }

        if (backRightPose != null
                && backRightPose.getPose() != null
                && backRightPose.getPoseEstimate().tagCount > 0
                && omega < 2) {
            DrivetrainConstants.drivetrain.addVisionMeasurement(backRightPose.getPose(), backRightPose.getTime());

        }
    }

    public void setupVisionImplantsAuto() {
        var driveState = DrivetrainConstants.drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        SmartDashboard.putNumber("heading Deg", headingDeg);
        double omega = Math.abs(Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond));
        frontRightCamera.SetRobotOrientation(headingDeg, 0, 0, 0, 0, 0);
        frontLeftCamera.SetRobotOrientation(headingDeg, 0, 0, 0, 0, 0);
        rearRightCamera.SetRobotOrientation(headingDeg, 0, 0, 0, 0, 0);

        PoseAllocate backRightPose = rearRightCamera.getRobotPose();
        PoseAllocate frontRightPose = frontRightCamera.getRobotPose();
        PoseAllocate frontLeftPose = frontLeftCamera.getRobotPose();
        if (frontRightPose != null
                && frontRightPose.getPose() != null
                && frontRightPose.getPoseEstimate().tagCount > 0
                && omega < 2) {
            DrivetrainConstants.drivetrain.addVisionMeasurement(frontRightPose.getPose(), frontRightPose.getTime());

        }

        if (frontLeftPose != null
                && frontLeftPose.getPose() != null
                && frontLeftPose.getPoseEstimate().tagCount > 0
                && omega < 2) {
            DrivetrainConstants.drivetrain.addVisionMeasurement(frontLeftPose.getPose(), frontLeftPose.getTime());

        }

    }

    private void configureBindings() {
        // Test Controller

        testController.povLeft()
                .onTrue(AutoBuilder.pathfindThenFollowPath(leftAutoPath, AutonConstraints.kPathConstraints));
        testController.povUp()
                .onTrue(AutoBuilder.pathfindThenFollowPath(centerAutoPath, AutonConstraints.kPathConstraints));
        testController.povDown()
                .onTrue(AutoBuilder.pathfindThenFollowPath(jackAutoPath, AutonConstraints.kPathConstraints));
        testController.a()
                .whileTrue(new ServoOut(climber));

        testController.leftTrigger().whileTrue(new RotateElevatorBasic(elevator.m_elevatorSpeed, elevator));

        testController
                .rightBumper().whileTrue(new RaiseSwanNeck(swanNeck, swanNeck.m_swanNeckPivotSpeed));
        // Triggers
        new Trigger(elevator::getElevatorAboveThreshold).and(elevator::getElevatorAtBelowL4)
                .and(RobotModeTriggers.teleop())
                .onTrue(new ConfigureHologenicDrive(driverController, DrivetrainConstants.drivetrain))
                .onFalse(ConfigureHologenicDrive(driverController, swerveSubsystem));

        new Trigger(elevator::getElevatorAtBarge).and(RobotModeTriggers.teleop())
                .onTrue(new ConfigureSlowDrive(driverController, DrivetrainConstants.drivetrain, 0.07))
                .onFalse(ConfigureHologenicDrive(driverController, swerveSubsystem));

        new Trigger(driverController.leftTrigger().and(frontLeftCamera::twoTagsDetected))
                .whileTrue(DrivetrainConstants.drivetrain.runOnce(() -> DrivetrainConstants.drivetrain
                        .resetRotation(new Rotation2d(frontLeftCamera.getMetatagYawRadians()))));

        new Trigger(driverController.leftTrigger().and(frontRightCamera::twoTagsDetected))
                .whileTrue(DrivetrainConstants.drivetrain.runOnce(() -> DrivetrainConstants.drivetrain
                        .resetRotation(new Rotation2d(frontRightCamera.getMetatagYawRadians()))));

        new Trigger(elevator::elevatorSwitchTriggered).and(RobotModeTriggers.teleop())
                .onTrue(new InstantCommand(elevator::setElevatorAtLimitHeight));
      
        // Driver
        driverController.start()
                .onTrue(new SequentialCommandGroup(
                        new ParallelDeadlineGroup(new WaitCommand(.25),
                                new SpinSwanWheels(swanNeckWheels, () -> IntakeMap.WHEEL_SPEED_SCORE * 2),
                                new HoldElevator(elevator)),
                        new RaiseSwanNeckPID(() -> ReefStops.SAFEANGLE, swanNeck).until(swanNeck::pidAtSetpoint),
                        new RotateElevatorDownPID(elevator).until(elevator::pidDownAtSetpoint)));

        // Auto Alignment Buttons
        driverController.leftBumper()
                .onTrue(new posePathfindToReef(frc.robot.commands.auton.posePathfindToReef.reefPole.LEFT,
                        DrivetrainConstants.drivetrain, frontRightCamera, frontLeftCamera));

        driverController.rightBumper()
                .onTrue(new posePathfindToReef(frc.robot.commands.auton.posePathfindToReef.reefPole.RIGHT,
                        DrivetrainConstants.drivetrain, frontRightCamera, frontLeftCamera));
        
        driverController.b()
        .onTrue(new ParallelCommandGroup(   
                    new retriveAlgae(elevator, swanNeck, swanNeckWheels, ElevatorMap.LOWALGAEROTATION))
        ).onFalse(new ParallelCommandGroup(new SpinSwanWheels(swanNeckWheels, () -> IntakeMap.ALGAE_WHEEL_SPEED),
                        new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.BARGEANGLE, swanNeck),
                        new SequentialCommandGroup(
                                new WaitCommand(.4).until(swanNeck::pidAtSetpoint),
                                new ParallelDeadlineGroup(new WaitCommand(.4),
                                        new MoveRobotCentricX(DrivetrainConstants.drivetrain, ()-> SwerveMap.BACKWARDS_SPEED)),
                                new ParallelCommandGroup(ConfigureHologenicDrive(driverController, swerveSubsystem),
                                        new RotateElevatorDownPID(elevator).until(elevator::pidDownAtSetpoint)))));
        driverController.povDown()
        .onTrue( new posePathfindToReef(frc.robot.commands.auton.posePathfindToReef.reefPole.CENTER,
        DrivetrainConstants.drivetrain, frontRightCamera, frontLeftCamera));
        driverController.x()
                .onTrue(new ParallelCommandGroup(   
                    new retriveAlgae(elevator, swanNeck, swanNeckWheels, ElevatorMap.HIGHALGAEROTATION)
        )).onFalse(new ParallelCommandGroup(new SpinSwanWheels(swanNeckWheels, () -> IntakeMap.ALGAE_WHEEL_SPEED),
                        new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.BARGEANGLE, swanNeck),
                        new SequentialCommandGroup(
                                new WaitCommand(.4).until(swanNeck::pidAtSetpoint),
                                new ParallelDeadlineGroup(new WaitCommand(.4),
                                        new MoveRobotCentricX(DrivetrainConstants.drivetrain, ()-> SwerveMap.BACKWARDS_SPEED)),
                                new ParallelCommandGroup(ConfigureHologenicDrive(driverController, swerveSubsystem),
                                        new RotateElevatorDownPID(elevator).until(elevator::pidDownAtSetpoint)))));

        driverController.leftTrigger()
                .whileTrue(new IntakeCoralSequence(swanNeck, swanNeckWheels, elevator));

        driverController.rightTrigger()
                .whileTrue(new IntakeAlgaeFromGround(swanNeck, elevator, swanNeckWheels))
                .onFalse(new SpinSwanWheels(swanNeckWheels, () -> IntakeMap.ALGAE_WHEEL_SPEED)
                        .alongWith(new SequentialCommandGroup(
                                new RaiseSwanNeckPID(() -> 0.1, swanNeck).until(swanNeck::pidAtSetpoint),
                                new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.SAFEANGLE, swanNeck))));

        // Climber Override

        // Panic Button
        // driverController.povRight()
        // .onTrue(new InstantCommand(()-> CommandScheduler.getInstance().cancelAll()));

        driverController.povRight()
                .whileTrue(new ParallelCommandGroup(new RaiseSwanNeckPID(() -> 0.0966, swanNeck),
                        new SpinSwanWheels(swanNeckWheels, () -> IntakeMap.ALGAE_WHEEL_SPEED)))
                .onFalse(new ParallelDeadlineGroup(new WaitCommand(2),
                        new SpinSwanWheels(swanNeckWheels, () -> -IntakeMap.ALGAE_WHEEL_SPEED)));

        driverController.povUp().and(()-> climber.getClimberActive())
                .whileTrue(new RaiseClimberBasic(() -> .15, climber)
                        .until(climber::climberPastZero))
                .whileFalse(new RaiseClimberBasic(() -> 0, climber));

        driverController.povUp().and(()-> climber.getClimberNotActivated())
                .onTrue(new climbingSequenceUp(climber));

        driverController.povLeft()
                .whileTrue(new PlaceBarge(elevator, swanNeck, swanNeckWheels))
                .onFalse(new SequentialCommandGroup(
                        new ParallelDeadlineGroup(new WaitCommand(2),
                                new RotateElevatorPID(elevator, () -> ElevatorMap.BARGEROTATION),
                                new SpinSwanWheels(swanNeckWheels, () -> -IntakeMap.ALGAE_WHEEL_SPEED - .2)),
                        new RaiseSwanNeckPIDAlgae(() -> IntakeMap.ReefStops.BARGEANGLE, swanNeck)
                                .until(swanNeck::pidAtSetpoint),
                        new RotateElevatorDownPID(elevator).until(elevator::pidDownAtSetpoint)));

        driverController.y().onTrue(new ScheduleCommand(elevator.runOnce(() -> {})).andThen(new ScheduleCommand(new SequentialCommandGroup(
            new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.AGAINSTREEFANGLE, swanNeck)
                    .until(swanNeck::pidAtSetpoint),
            new RotateElevatorNextLevelUp(elevator), new ParallelCommandGroup(new HoldElevator(elevator),
                    new RaiseSwanNeckToScoringAngle(swanNeck, elevator))))));
                

        driverController.a()
                .onTrue(new ScheduleCommand(elevator.runOnce(() -> {})).andThen(new ScheduleCommand( new SequentialCommandGroup(
                    new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.AGAINSTREEFANGLE, swanNeck)
                            .until(swanNeck::pidAtSetpoint),
                    new RotateElevatorNextLevelDown(elevator), new ParallelCommandGroup(new HoldElevator(elevator),
                            new RaiseSwanNeckToScoringAngle(swanNeck, elevator))))));
                   

    }

    private void setupEventTriggers() {
        new EventTrigger("L3Infinite").whileTrue(new RotateElevatorSafePID(elevator));
        new EventTrigger("FeedThenL3").whileTrue(new FeedThenL3(elevator, swanNeck, swanNeckWheels));
    }

    private void setupNamedCommands() {
        NamedCommands.registerCommand("Field Relative",
                DrivetrainConstants.drivetrain.runOnce(() -> DrivetrainConstants.drivetrain.seedFieldCentric()));
        NamedCommands.registerCommand("L4", new PlaceLFour(elevator, swanNeck, swanNeckWheels)
                .andThen(new RotateElevatorSafePID(elevator).until(elevator::pidSafeAtSetpoint)));
        NamedCommands.registerCommand("L4Fast", new PlaceLFourAuton(elevator, swanNeck, swanNeckWheels));
        NamedCommands.registerCommand("MetatagRelativeYaw",
                DrivetrainConstants.drivetrain.runOnce(() -> DrivetrainConstants.drivetrain
                        .resetRotation(new Rotation2d(frontLeftCamera.getMetatagYawRadians()))));
        NamedCommands.registerCommand("MetatagRelativeYawSafe",
                DrivetrainConstants.drivetrain
                        .runOnce(() -> DrivetrainConstants.drivetrain
                                .resetRotation(new Rotation2d(frontLeftCamera.getMetatagYawRadians())))
                        .onlyIf(frontLeftCamera::twoTagsDetected));
        NamedCommands.registerCommand("MetatagRelativeYawRightSafe",
                DrivetrainConstants.drivetrain
                        .runOnce(() -> DrivetrainConstants.drivetrain
                                .resetRotation(new Rotation2d(frontRightCamera.getMetatagYawRadians())))
                        .onlyIf(frontRightCamera::twoTagsDetected));
        NamedCommands.registerCommand("MetatagRelativeYawRight",
                DrivetrainConstants.drivetrain.runOnce(() -> DrivetrainConstants.drivetrain
                        .resetRotation(new Rotation2d(frontRightCamera.getMetatagYawRadians()))));
        NamedCommands.registerCommand("ElevatorDown",
                new SequentialCommandGroup(
                        new RaiseSwanNeckPID(() -> ReefStops.SAFEANGLE, swanNeck).until(swanNeck::pidAtSetpoint),
                        new RotateElevatorDownPID(elevator).until(elevator::pidDownAtSetpoint)));
        NamedCommands.registerCommand("Feed", new IntakeCoralSequence(swanNeck, swanNeckWheels, elevator));
        NamedCommands.registerCommand("ZeroMechanisms", zeroMechanisms);
        NamedCommands.registerCommand("L3Infinite", new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.SAFEANGLE, swanNeck)
                .until(swanNeck::pidAtSetpoint).andThen(new RotateElevatorPID(elevator, () -> ElevatorMap.L2ROTATION)));
        NamedCommands.registerCommand("L2", new PlaceLTwo(elevator, swanNeck, swanNeckWheels));
        NamedCommands.registerCommand("L4Height",
                new RaiseSwanNeckPID(() -> ReefStops.SAFEANGLE, swanNeck).until(swanNeck::pidAtSetpoint)
                        .andThen(new RotateElevatorPID(elevator, () -> ElevatorMap.L4ROTATION)
                                .until(elevator::pidAtSetpoint))
                        .andThen(new ParallelCommandGroup(new RotateElevatorPID(elevator, () -> ElevatorMap.L4ROTATION),
                                new RaiseSwanNeckPID(() -> ReefStops.L4ANGLE, swanNeck))));
        NamedCommands.registerCommand("SpinRelease", new ParallelDeadlineGroup(new WaitCommand(.2),
                new SpinSwanWheels(swanNeckWheels, () -> IntakeMap.WHEEL_SPEED_SCORE - .25)));
        NamedCommands.registerCommand("L4SpinRelease",
                new ParallelDeadlineGroup(
                        new ParallelDeadlineGroup(new WaitCommand(.2),
                                new SpinSwanWheels(swanNeckWheels, () -> IntakeMap.WHEEL_SPEED_SCORE * 2)),
                        new RotateElevatorPID(elevator, () -> ElevatorMap.L4ROTATION)));
        NamedCommands.registerCommand("LowAlgae",
                new retriveAlgae(elevator, swanNeck, swanNeckWheels, ElevatorMap.LOWALGAEROTATION));
        NamedCommands.registerCommand("HighAlgae",
                new retriveAlgae(elevator, swanNeck, swanNeckWheels, ElevatorMap.HIGHALGAEROTATION));
        NamedCommands.registerCommand("UpToBarge", new PlaceBarge(elevator, swanNeck, swanNeckWheels));
        NamedCommands.registerCommand("ScoreBarge",
                new ParallelDeadlineGroup(new WaitCommand(1),
                        new RotateElevatorPID(elevator, () -> ElevatorMap.BARGEROTATION),
                        new SpinSwanWheels(swanNeckWheels, () -> -IntakeMap.ALGAE_WHEEL_SPEED)));
        NamedCommands.registerCommand("DownFromBarge",
                new SequentialCommandGroup(
                        new RaiseSwanNeckPIDAlgae(() -> IntakeMap.ReefStops.BARGEANGLE, swanNeck)
                                .until(swanNeck::pidAtSetpoint),
                        new RotateElevatorDownPID(elevator).until(elevator::pidDownAtSetpoint)));
        NamedCommands.registerCommand("L1", new PlaceLOne(elevator, swanNeck, swanNeckWheels));
        NamedCommands.registerCommand("L3Height", new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.SAFEANGLE, swanNeck)
                .until(swanNeck::pidAtSetpoint).andThen(new RotateElevatorPID(elevator, () -> ElevatorMap.L3ROTATION)));
        NamedCommands.registerCommand("SuckInAlgae",
                new SpinSwanWheels(swanNeckWheels, () -> IntakeMap.ALGAE_WHEEL_SPEED)
                        .alongWith(new RaiseSwanNeckPID(() -> IntakeMap.ReefStops.SAFEANGLE, swanNeck)));
        NamedCommands.registerCommand("MoveBackwards", new MoveRobotCentricX(DrivetrainConstants.drivetrain, ()-> SwerveMap.FAST_BACKWARDS_SPEED));
    }

    
    public static Command repeatWhile(Supplier<Command> cmd, BooleanSupplier cond) {
       boolean[] atEnd = { false };
      return Commands.runOnce(() -> { atEnd[0] = true; })
        .andThen(Commands.deferredProxy(cmd::get))
        .alongWith(Commands.runOnce(() -> { atEnd[0] = false; }))
        .finallyDo(() -> { atEnd[0] = true; })
        .repeatedly()
        .until(() -> atEnd[0] && cond.getAsBoolean());
    }

    public void setupPaths() {
       //code project - add option of "CompRight3L4Robust", putting together the auto sections and detours in the same way as seen below

        autonChooser.addOption("CompLeft3L4Robust",
        new SequentialCommandGroup(
            Left3L4Part1,
           AutoBuilder.buildAuto("Left3L4Part1Detour").onlyIf(()-> !swanNeck.getCoralLoaded()),
           AutoBuilder.buildAuto("Left3L4Part1Detour").onlyIf(()-> !swanNeck.getCoralLoaded()),
           AutoBuilder.buildAuto("Left3L4Part1Detour").onlyIf(()-> !swanNeck.getCoralLoaded()),
            Left3L4Part2,
           AutoBuilder.buildAuto("Left3L4Part2Detour").onlyIf(()-> !swanNeck.getCoralLoaded()),
           AutoBuilder.buildAuto("Left3L4Part2Detour").onlyIf(()-> !swanNeck.getCoralLoaded()),
           AutoBuilder.buildAuto("Left3L4Part2Detour").onlyIf(()-> !swanNeck.getCoralLoaded()),
            Left3L4Part3
        )
    );

       
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

    public void setupElevator() {
        // commandChooser.addOption("GoingUP", new GoUpCommand(elevator, 0.1, 1.0));

        Shuffleboard.getTab(Subsystems.SWERVE_DRIVE.getNetworkTable()).add("Command Chooser", commandChooser)
                .withSize(2, 1)
                .withProperties(Map.of("position", "0, 2"));
    }

    // DO NOT REMOVE
    public SubsystemABS[] SafeGuardSystems() {
        return new SubsystemABS[] {
                swerveSubsystem,
                frontRightCamera,
                // rearLeftCamera,
                // rearRightCamera,
                frontLeftCamera

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
