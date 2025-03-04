// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.lift.RotateElevatorL3PID;
import frc.robot.commands.swanNeck.IntakeCoralSequence;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.swanNeck.SwanNeck;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedThenL3 extends SequentialCommandGroup {
  Lift m_elevator;
  SwanNeck m_gooseNeck;
  /** Creates a new FeedThenL3. */
  public FeedThenL3(Lift lift, SwanNeck swanNeck) {
    m_elevator = lift;
    m_gooseNeck = swanNeck;
    addCommands(new IntakeCoralSequence(m_gooseNeck), new RotateElevatorL3PID(lift));
  }
}
