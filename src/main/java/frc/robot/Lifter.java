package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;

/**
 * An instance of the lifter/elevator which may influence cargo and hatches.
 * It should obey a "move to position" policy, with multiple target levels.
 *
 * Level 0: on ground, for collecting cargo
 * Level 1-3: in air, for scoring cargo on rocket levels
 *
 *
 */
public class Lifter {

    private WPI_TalonSRX talon;

    private double target = 0;

    /**
     * The elevation (inches) of the center of the lowest rocket cargo port
     */
    private static final double PORT_LOW = 27.5;

    /**
     * The distance (inches) between the centers of each rocket cargo port
     */
    private static final double PORT_INTERVAL = 28;

    /**
     * The elevation (inches) of the center of a cargo ball when it is in
     * the lowest position within the inner claw - i.e. just collected
     */
    private static final double LOWEST_CARGO_CENTER = 12; //TODO: evaluate

    /**
     * Encoder ticks per each inch the lifter travels vertically
     */
    private static final double TICKS_PER_VERTICAL_INCH = 1000; // TODO: evaluate

    /**
     * The elevation (ticks) of the point at which we want to eject cargo into the cargo ship.
     */
    public static final double CARGO_DEPOSIT_HEIGHT = 40 * TICKS_PER_VERTICAL_INCH; //TODO: evaluate

    public Lifter(WPI_TalonSRX talon) {
        this.talon = talon;

        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        talon.setSensorPhase(true);

        talon.setInverted(false);

        talon.configNominalOutputForward(0);
        talon.configNominalOutputReverse(0);
        talon.configPeakOutputForward(0.5);
        talon.configPeakOutputReverse(-0.5);

        talon.configAllowableClosedloopError(0, 16);

        talon.enableCurrentLimit(true);
        talon.configContinuousCurrentLimit(20);
        talon.configPeakCurrentLimit(0);

        talon.config_kP(0, 0.5);
        talon.config_kI(0, 0);
        talon.config_kD(0, 0);
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 30);
        talon.setSelectedSensorPosition(0);
    }

//    public void moveToLevel(int level) {
//        // TODO: redo, remap levels
//        if (level == 0) {
//            this.moveToEncoder(0);
//        } else {
//            double target = ((level-1) * PORT_INTERVAL + PORT_LOW - LOWEST_CARGO_CENTER) * TICKS_PER_VERTICAL_INCH;
//            this.moveToEncoder((int) target);
//        }
//    }

    public void run() {
        talon.set(ControlMode.Position, target);
    }

    public void setTarget(double target) {
        this.target = target;
    }
}