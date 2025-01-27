// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.camera.Camera;
import frc.robot.utils.AutoPathFinder;
import frc.robot.utils.PathPair;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class direction extends Command {
  private Camera camera;

  private static final PathPair[] PATHS = {
      // //Coral Stations
      // new PathPair(1, 13, "thispathwasntmadeyetdontrunthis", "thispathwasntmadeyetdontrunthis"),
      // new PathPair(2, 12, "thispathwasntmadeyetdontrunthis", "thispathwasntmadeyetdontrunthis"),
      // //Processor
      // new PathPair(3, 16, "thispathwasntmadeyetdontrunthis", "thispathwasntmadeyetdontrunthis"),
      // //Net tags
      // new PathPair(4, 15, "thispathwasntmadeyetdontrunthis", "thispathwasntmadeyetdontrunthis"),
      // new PathPair(5, 14, "thispathwasntmadeyetdontrunthis", "thispathwasntmadeyetdontrunthis"),
      //Reef Paths
      new PathPair(6, 19, "66alignRight", "66alignRight"),
      new PathPair(7, 18, "56alignRight", "56alignRight"),
      new PathPair(8, 17, "46alignLeft", "46alignRight"),
      new PathPair(9, 22, "36alignLeft", "36alignRight"),
      new PathPair(10, 21, "26alignLeft", "26alignRight"),
      new PathPair(11, 20, "16alignLeft", "16alignRight")
  };

  /** Creates a new direction. */
  public direction(Camera camera) {
    this.camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (camera.getLastseenAprilTag() == -1) {
      return;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(camera.getLastseenAprilTag());
    SmartDashboard.putNumber("TagID----------------", camera.getLastseenAprilTag());
    for (PathPair path : PATHS) {
      if (path.tagToPath(camera.getLastseenAprilTag())) {
        AutoPathFinder.GotoPath(path.getRightPath());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
