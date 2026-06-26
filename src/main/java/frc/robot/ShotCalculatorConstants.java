// Constants for shot calculation
package frc.robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LoggedTunableNumber;

/** Field poses + align tuning. Flip handled for blue. */
public class ShotCalculatorConstants {
    public static final Distance shooterSideOffset = Units.Inches.of(0); // distance that the shooter is offset laterally from the robot center. Positive is to the right, negative is to the left. Tune this value based on your robot's actual shooter position.

    public static final Transform2d shooterTransform =
        new Transform2d(Units.Inches.of(0.0), shooterSideOffset, new Rotation2d());
    private static final LoggedTunableNumber kHubLateralScale =
        new LoggedTunableNumber("Drive/Hub/LateralScale", 0.2, true);

    public static double getHubLateralScale() {
        return SmartDashboard.getNumber("Drive/Hub/LateralScale", kHubLateralScale.get());
    }

    // Air time lookup table for SOTM compensation - maps distance (meters) to air time (seconds)
    // Tune these values based on your shooter's projectile speed
    private static final InterpolatingDoubleTreeMap kAirTimeMap = new InterpolatingDoubleTreeMap();
    private static final LoggedTunableNumber kAirTime1m = new LoggedTunableNumber("Drive/SOTM/AirTime/1m", 0.2, true);
    private static final LoggedTunableNumber kAirTime2m = new LoggedTunableNumber("Drive/SOTM/AirTime/2m", 0.3, true);
    private static final LoggedTunableNumber kAirTime3m = new LoggedTunableNumber("Drive/SOTM/AirTime/3m", 0.4, true);
    private static final LoggedTunableNumber kAirTime4m = new LoggedTunableNumber("Drive/SOTM/AirTime/4m", 0.5, true);
    private static final LoggedTunableNumber kAirTime5m = new LoggedTunableNumber("Drive/SOTM/AirTime/5m", 0.6, true);
    private static final LoggedTunableNumber kAirTime6m = new LoggedTunableNumber("Drive/SOTM/AirTime/6m", 0.7, true);

    /**
     * Get air time for a given distance using interpolation map.
     * @param distanceMeters Distance to target in meters
     * @return Estimated air time in seconds
     */
    public static double getAirTime(double distanceMeters) {
        // Update map with current tunable values each call
        kAirTimeMap.put(1.0, kAirTime1m.get());
        kAirTimeMap.put(2.0, kAirTime2m.get());
        kAirTimeMap.put(3.0, kAirTime3m.get());
        kAirTimeMap.put(4.0, kAirTime4m.get());
        kAirTimeMap.put(5.0, kAirTime5m.get());
        kAirTimeMap.put(6.0, kAirTime6m.get());
        return kAirTimeMap.get(distanceMeters);
    }
}