// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.constants.RobotMap;
// import frc.robot.subsystems.climber.Climber;
// import frc.robot.utils.DrivetrainConstants;

// WORK IN PROGRESS


// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class climbingSequence extends SequentialCommandGroup {
//   Climber m_climber;

//   /** Creates a new climbingSequence. */
//   public climbingSequence(Climber climber) {
//     m_climber = climber;
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(new raiseClimber(m_climber, RobotMap.ClimberMap.CLIMBER_UP_ANGLE), DrivetrainConstants.drivetrain.applyRequest(DrivetrainConstants.robotDrive.withVelocityX(.1) ));
//   }
// }
// clifford is a cool dude
