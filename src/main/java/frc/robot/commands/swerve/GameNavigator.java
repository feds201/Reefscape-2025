// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.vision.camera.Camera;
import frc.robot.utils.AutoPathFinder;
import frc.robot.utils.PathPair;
import frc.robot.utils.VisionABC;

public class GameNavigator extends AutoPathFinder {
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

  public static Command GoLeft(Camera camera) {
    SmartDashboard.putNumber("TagID", camera.getLastseenAprilTag());
    if (camera.getLastseenAprilTag() == -1) {
      return new ParallelCommandGroup().withTimeout(2);
      //return null;
      /* returning null causes an error, -1 acts as null in this case. */
    }

    for (PathPair path : PATHS) {
      if (path.tagToPath(camera.getLastseenAprilTag() )) {

        return new ParallelCommandGroup(GotoPath(path.getLeftPath()));
      }

    }
    return new ParallelCommandGroup().withTimeout(2);
    //Do nothing (in case something went wrong when traversing the PathPair list)
  }

  public static Command GoRight(Camera camera) {
    SmartDashboard.putNumber("TagID", camera.getLastseenAprilTag() );
    if (camera.getLastseenAprilTag()  == -1) {
      return new ParallelCommandGroup().withTimeout(2);
        //return null;
      /* returning null causes an error, -1 acts as null in this case. */
    }

    for (PathPair path : PATHS) {
      if (path.tagToPath(camera.getLastseenAprilTag() )) {

        return new ParallelCommandGroup(GotoPath(path.getRightPath()));
      }

    }
    return new ParallelCommandGroup().withTimeout(2);
    //Do nothing (in case something went wrong when traversing the PathPair list)
  }
}
