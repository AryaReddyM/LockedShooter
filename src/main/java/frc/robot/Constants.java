package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : 
        RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY;

    public static enum Mode { 
        REAL,
        SIM,
        REPLAY
    }

    public static final RobotType robot = RobotType.PRIMARY;
    public static enum RobotType { PRIMARY, PRACTICE }

    public static final String kCANivoreBus = "canivore";
}
