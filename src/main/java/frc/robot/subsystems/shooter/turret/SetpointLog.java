package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class SetpointLog {
    public double shooterRPS = 0.0;
    public double shooterStage1RPS = 14.4;
    public double turretRadiansFromCenter = 0.0;
    public double turretFF = 0.0;
    public double hoodRadians = 0.0;
    public double hoodFF = 0.0;
    public double height = 0.0;
    public boolean isValid = true;
}
