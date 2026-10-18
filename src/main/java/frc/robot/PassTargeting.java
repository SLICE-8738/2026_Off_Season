package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Pass targeting, used ONLY by the pass path (AutoAlignTrench + Shooter.distanceFromTrench).
 * It is deliberately self-contained: it does not touch the drivetrain, the heading PID,
 * or anything the hub shot uses.
 *
 * Two different points come out of here, on purpose:
 *   - AIM point:           OUR alliance's *_PASS_* point (left or right) the robot faces.
 *   - FLYWHEEL reference:  OUR alliance's *_TRENCH_* point on the SAME side. The
 *                          PASSING_MAP RPM table was tuned against trench distances, so
 *                          flywheel speed keeps being computed from those.
 * Both follow the same left/right choice, so aim and RPM can't disagree about the side.
 */
public final class PassTargeting {
    private PassTargeting() {}

    /** Switch sides only if the other pass point is closer by MORE than this (meters). */
    private static final double SWITCH_HYSTERESIS_METERS = 0.5;

    /** The pass point we are currently aiming at, or null before the first pick. */
    private static Pose2d current = null;

    /** Which side the current pick is on (true = left). Only valid when current != null. */
    private static boolean currentIsLeft = false;

    /** Forget the previous pick. Called when a new pass press begins. */
    public static void reset() {
        current = null;
    }

    /**
     * Re-evaluates the AIM point from the robot's current position and returns it.
     * Call once per loop from AutoAlignTrench.execute().
     */
    public static Pose2d update(boolean isBlue, Translation2d robot) {
        // Poses are always blue-origin, so we just use OUR alliance's constants.
        Translation2d left  = isBlue ? Constants.AlignTargets.BLUE_PASS_LEFT
                                     : Constants.AlignTargets.RED_PASS_LEFT;
        Translation2d right = isBlue ? Constants.AlignTargets.BLUE_PASS_RIGHT
                                     : Constants.AlignTargets.RED_PASS_RIGHT;

        double dLeft  = robot.getDistance(left);
        double dRight = robot.getDistance(right);

        boolean wasNull = (current == null);
        boolean wasLeft = currentIsLeft;

        if (wasNull) {
            // First pick of this press: just take the nearer one.
            currentIsLeft = dLeft < dRight;
        } else {
            // Already have a side: only switch if the other is clearly closer.
            double dCurrent = currentIsLeft ? dLeft : dRight;
            double dOther   = currentIsLeft ? dRight : dLeft;
            if (dOther + SWITCH_HYSTERESIS_METERS < dCurrent) {
                currentIsLeft = !currentIsLeft;
            }
        }

        if (wasNull || currentIsLeft != wasLeft) {
            current = new Pose2d(currentIsLeft ? left : right, Rotation2d.kZero);
        }
        return current;
    }

    /** Current AIM point (picks one if none yet). */
    public static Pose2d get(boolean isBlue, Translation2d robot) {
        return (current != null) ? current : update(isBlue, robot);
    }

    /**
     * The point the FLYWHEEL distance is measured to: OUR alliance's trench point on the
     * same side as the current aim point. Used by Shooter.distanceFromTrench().
     */
    public static Pose2d getFlywheelReference(boolean isBlue, Translation2d robot) {
        get(isBlue, robot); // make sure a side has been chosen
        Translation2d trench;
        if (isBlue) {
            trench = currentIsLeft ? Constants.AlignTargets.BLUE_TRENCH_LEFT
                                   : Constants.AlignTargets.BLUE_TRENCH_RIGHT;
        } else {
            trench = currentIsLeft ? Constants.AlignTargets.RED_TRENCH_LEFT
                                   : Constants.AlignTargets.RED_TRENCH_RIGHT;
        }
        return new Pose2d(trench, Rotation2d.kZero);
    }
}
