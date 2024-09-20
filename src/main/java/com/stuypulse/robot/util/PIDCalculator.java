package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;
import com.stuypulse.stuylib.streams.numbers.filters.IFilterGroup;
import com.stuypulse.stuylib.streams.numbers.filters.TimedMovingAverage;
import com.stuypulse.stuylib.util.StopWatch;

/**
 * This is a Bang-Bang controller that while controlling the robot, will be able to calculate the
 * values for the PID controller. It does this by taking the results of oscillations, then creating
 * a PIDController withe the correct values once the oscillations have been measured.
 *
 * @author Sam (sam.belliveau@gmail.com)
 */
public class PIDCalculator extends Controller {

    // Maximum amount of time between update commands before the calculator
    // resets its measurements
    private static final double kMaxTimeBeforeReset = 0.5;
    private final StopWatch stopWatch = new StopWatch();

    // The minimum period length that will be accepted as a valid period
    private static final double kMinPeriodTime = 0.05;

    // The filter easuring the period and amplitude
    private static IFilter getMeasurementFilter() {
        // This is a mix between accuracy and speed of updating.
        // Takes about 6 periods to get accurate results
        return new IFilterGroup(new TimedMovingAverage(30));
    }

    // The speed that the bang bang controller will run at
    private Number mControlSpeed;
    private double mCurrentSpeed;

    // The results of the period and amplitude
    private double mPeriod;
    private double mAmplitude;

    // The filters used to average the period and amplitudes
    private IFilter mPeriodFilter;
    private IFilter mAmplitudeFilter;

    // Timer that keeps track of the length of a period
    private StopWatch mPeriodTimer;

    // The min and max of the wave
    private double mLocalMax;

    // Whether or not the system will measure the oscillation
    private boolean mRunning;

    /** @param speed motor output for bang bang controller */
    public PIDCalculator(Number speed) {
        mControlSpeed = speed;
        mCurrentSpeed = mControlSpeed.doubleValue();

        mPeriod = 0;
        mPeriodFilter = getMeasurementFilter();

        mAmplitude = 0;
        mAmplitudeFilter = getMeasurementFilter();

        mPeriodTimer = new StopWatch();
        mLocalMax = 0;

        mRunning = false;
    }

    /**
     * @param speed sets speed for motor output of controller
     * @return the calculated result from the PIDController
     */
    public PIDCalculator setControlSpeed(Number speed) {
        mControlSpeed = SmartNumber.setNumber(mControlSpeed, speed);
        return this;
    }

    /**
     * Calculate the value that the controller wants to move at while calculating the values for the
     * PIDController
     *
     * @param error the error that the controller will use
     * @return the calculated result from the controller
     */
    protected double calculate(double setpoint, double measurement) {
        double error = setpoint - measurement;
        // If there is a gap in updates, then disable until next period
        if (stopWatch.getTime() > kMaxTimeBeforeReset) {
            mRunning = false;
        }
        else {
            stopWatch.reset();
        }

        // Check if we crossed 0, ie, time for next update
        if (Math.signum(mCurrentSpeed) != Math.signum(error)) {
            // Update the controller
            mCurrentSpeed = mControlSpeed.doubleValue() * Math.signum(error);

            // Get period and amplitude
            double period = mPeriodTimer.reset() * 2.0;
            double amplitude = mLocalMax;

            // If we are running and period is valid, record it
            if (mRunning && kMinPeriodTime < period) {
                mPeriod = mPeriodFilter.get(period);
                mAmplitude = mAmplitudeFilter.get(amplitude);
            }

            // Reset everything
            mLocalMax = 0;
            mRunning = true;
        }

        // Calculate amplitude by recording maximum
        mLocalMax = Math.max(Math.abs(mLocalMax), Math.abs(error));

        // Return bang bang control
        return mCurrentSpeed;
    }

    /**
     * Adjusted Amplitude of Oscillations
     *
     * @return Get calculated K value for PID value equation
     */
    public double getK() {
        return (4.0 * mControlSpeed.doubleValue()) / (Math.PI * mAmplitude);
    }

    /**
     * Period of Oscillations
     *
     * @return Get calculated T value for PID value equation
     */
    public double getT() {
        return mPeriod;
    }

    /**
     * @param kP p multiplier when calculating values
     * @param kI p multiplier when calculating values
     * @param kD p multiplier when calculating values
     * @return calculated PID controller based off of measurements
     */
    private PIDController getPIDController(double kP, double kI, double kD) {
        kP = Math.max(kP, 0.0);
        kI = Math.max(kI, 0.0);
        kD = Math.max(kD, 0.0);

        if (mAmplitude > 0) {
            double t = getT();
            double k = getK();

            return new PIDController(kP * (k), kI * (k / t), kD * (k * t));
        } else {
            return new PIDController(-1, -1, -1);
        }
    }

    /** @return calculated PID controller based off of measurements */
    public PIDController getPIDController() {
        return getPIDController(0.6, 1.2, 3.0 / 40.0);
    }

    /** @return calculated PI controller based off of measurements */
    public PIDController getPIController() {
        return getPIDController(0.45, 0.54, -1);
    }

    /** @return calculated PD controller based off of measurements */
    public PIDController getPDController() {
        return getPIDController(0.8, -1, 1.0 / 10.0);
    }

    /** @return calculated P controller based off of measurements */
    public PIDController getPController() {
        return getPIDController(0.5, -1, -1);
    }

    /** @return information about this PIDController */
    public String toString() {
        return "(K: "
                + round(getK(), 4)
                + ", T: "
                + round(getT(), 4)
                + ") "
                + getPIDController().toString();
    }

    /**
     * Round a double by a certain amount of sigfigs in base 10
     *
     * @param n number to round
     * @param sigfigs amount of sigfigs to round it to
     * @return rounded number
     */
    public static double round(double n, int sigfigs) {
        // The value 0 returns nan if not accounted for
        if (n == 0.0) {
            return 0.0;
        }

        // Digit place that number starts at
        int digits = (int) Math.floor(Math.log10(Math.abs(n)));

        // Amount to multiply before multiplying based on
        // the sigfigs and digits in the number
        double mul = fpow(10.0, sigfigs - digits);

        // Round number by the multiplier calculated
        return Math.round(n * mul) / mul;
    }

    /**
     * fpow (fast pow), is a pow function that takes in an integer for the exponent. This allows it
     * to be much faster on repeated calls due to the fact that it does not need to deal with
     * fractional exponents.
     *
     * @param base base of the power
     * @param exp integer exponent of power
     * @return result of calculation
     */
    public static double fpow(double base, int exp) {
        // Output of the fpow function
        double out = 1.0;

        // If the exponent is negative, divide instead of multiply
        if (exp < 0) {
            // Flip exponent to make calculations easier
            exp = -exp;

            // Fast integer power algorithm
            while (exp > 0) {
                if ((exp & 1) == 1) {
                    out /= base;
                }
                base *= base;
                exp >>= 1;
            }
        } else {
            // Fast integer power algorithm
            while (exp > 0) {
                if ((exp & 1) == 1) {
                    out *= base;
                }
                base *= base;
                exp >>= 1;
            }
        }

        // Return
        return out;
    }

    /*********************/
    /*** Sendable Data ***/
    /*********************/

    // @Override
    // public void initSendable(SendableBuilder builder) {
    //     super.initSendable(builder);

    //     builder.addDoubleProperty("(PIDCalculator) Period [T]", this::getT, x -> {});
    //     builder.addDoubleProperty("(PIDCalculator) Adjusted Amplitude [K]", this::getK, x -> {});

    //     builder.addDoubleProperty(
    //             "(PIDCalculator) Calculated kP", () -> this.getPIDController().getP(), x -> {});
    //     builder.addDoubleProperty(
    //             "(PIDCalculator) Calculated kI", () -> this.getPIDController().getI(), x -> {});
    //     builder.addDoubleProperty(
    //             "(PIDCalculator) Calculated kD", () -> this.getPIDController().getD(), x -> {});
    // }
}
