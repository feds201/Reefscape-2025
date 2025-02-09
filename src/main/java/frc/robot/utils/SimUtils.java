package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;

public class SimUtils {
    public static final Mode simMode = Mode.SIM_ROBOT;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL_ROBOT : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL_ROBOT,

        /** Running a physics simulator. */
        SIM_ROBOT,

        /** Replaying from a log file. */
        REPLAY
    }
}
