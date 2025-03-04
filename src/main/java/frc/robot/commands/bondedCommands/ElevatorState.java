// package frc.robot.commands.bondedCommands;

// import org.dyn4j.geometry.Rotation;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.RobotMap.ElevatorMap;
// import frc.robot.subsystems.Elevator.*;

// public class ElevatorState extends Command {
//     // Predefined elevator rotation positions
//     public static final Rotation2d L1_ROTATION = new Rotation2d(ElevatorMap.L1ROTATION);
//     public static final Rotation2d L2_ROTATION = new Rotation2d(ElevatorMap.L2ROTATION);
//     public static final Rotation2d L3_ROTATION = new Rotation2d(ElevatorMap.L3ROTATION);
//     public static final Rotation2d L4_ROTATION = new Rotation2d(ElevatorMap.L4ROTATION);

//     private final Lift elevator;
//     private final Rotation2d targetRotation;
//     private final double speed;
//     private final double tolerance;

//     /**
//      * Creates a command to move the elevator to a target rotation.
//      *
//      * @param elevator The elevator subsystem
//      * @param targetRotation The target rotation to reach
//      * @param speed The maximum speed to use (0.0 to 1.0)
//      * @param tolerance Position tolerance for considering the target reached
//      */
//     public ElevatorState(Lift elevator, Rotation2d
//      targetRotation, double speed, double tolerance) {
//         this.elevator = elevator;
//         this.targetRotation = targetRotation;
//         this.speed = speed;
//         this.tolerance = tolerance;
//         addRequirements(elevator);
//     }

//     /**
//      * Creates a command to move the elevator to a predefined target rotation with default speed and tolerance.
//      *
//      * @param elevator The elevator subsystem
//      * @param targetRotation The target rotation to reach
//      */
//     public ElevatorState(Lift elevator, Rotation2d targetRotation) {
//         this(elevator, targetRotation, 0.5, 0.05);
//     }

//     /**
//      * Factory method to create an L1 position command
//      */
//     public static ElevatorState createL1Command(Lift elevator) {
//         return new ElevatorState(elevator, L1_ROTATION);
//     }

//     /**
//      * Factory method to create an L2 position command
//      */
//     public static ElevatorState createL2Command(Lift elevator) {
//         return new ElevatorState(elevator, L2_ROTATION);
//     }

//     /**
//      * Factory method to create an L3 position command
//      */
//     public static ElevatorState createL3Command(Lift elevator) {
//         return new ElevatorState(elevator, L3_ROTATION);
//     }

//     /**
//      * Factory method to create an L4 position command
//      */
//     public static ElevatorState createL4Command(Lift elevator) {
//         return new ElevatorState(elevator, L4_ROTATION);
//     }

//     @Override
//     public void initialize() {
//         elevator.setPIDTarget(targetRotation.getRotations());
//         elevator.setPIDTolerance(tolerance);
//     }

//     @Override
//     public void execute() {
//         elevator.rotateElevatorPID();
//         SmartDashboard.putNumber("Elevator Target", targetRotation.getRadians());
//     }

//     @Override
//     public void end(boolean interrupted) {
//         elevator.setMotorSpeed(0);
//     }

//     @Override
//     public boolean isFinished() {
//         return elevator.pidAtSetpoint();
//     }
// }
