// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;

public class climbingSequenceUp extends SequentialCommandGroup {
    private Climber m_climber;

    /** Creates a new climbingSequence. */
    public climbingSequenceUp(Climber climber) {
        m_climber = climber;
        
        addCommands(new ParallelDeadlineGroup(new WaitCommand(5), new InstantCommand(()-> m_climber.setServoIn())), new RaiseClimberBasic(()-> .08, climber).until(m_climber :: internalEncoderPastThreshold), new WaitCommand(.1), new RaiseClimberBasic(()-> .085, climber)
        .until(m_climber :: climberAtStraight), new InstantCommand(()-> m_climber.setClimberActivated())
        );
    }
}
