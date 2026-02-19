package frc.robot.subsystems.climb;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;


public class BeamBreakerTOF implements BeamBreakerIO{
    private TimeOfFlight sensor;

    public BeamBreakerTOF(int number) {
        int sensorId = 0;

        switch(number) {
            case 1:
                sensorId = ClimbConstants.kBeamBreakerIdOne;
                break;
            case 2:
                sensorId = ClimbConstants.kBeamBreakerIdTwo;
                break;
            default:
                sensorId = 0;
                break;
        }
        this.sensor = new TimeOfFlight(sensorId);
        this.sensor.setRangingMode(RangingMode.Medium, 24);
    }

    public void updateInputs(BeamBreakerInputs inputs) {
        inputs.distance = getDistance();
    }

    public double getDistance() {
        return sensor.getRange()/1000; /// in meters
    }
}
