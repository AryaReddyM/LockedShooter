package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.base.MotorIO;
import frc.robot.subsystems.base.ServoMotorSubsystem;
import frc.robot.subsystems.base.Setpoint;

public class Elevator extends ServoMotorSubsystem {
    public static final Setpoint STOW = Setpoint.motionMagicPosition(ElevatorConstants.kStowMeters);
    public static final Setpoint LOW = Setpoint.motionMagicPosition(ElevatorConstants.kLowMeters);
    public static final Setpoint MID = Setpoint.motionMagicPosition(ElevatorConstants.kMidMeters);
    public static final Setpoint HIGH = Setpoint.motionMagicPosition(ElevatorConstants.kHighMeters);

    public Elevator(MotorIO io) {
        super(io, "Elevator", ElevatorConstants.kToleranceMeters);
    }

    public Command goToHeight(Setpoint height) {
        return holdSetpoint(height, () -> ElevatorConstants.kG);
    }

    public double getHeightMeters() {
        return getPositionRad();
    }
}
