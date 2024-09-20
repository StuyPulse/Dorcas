package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

/**
 * Output filter meant for a distance PID Controller that works
 * at the same time as a turning PID controller.
 * 
 * Adjusts the speed output of the distance controller based on how
 * unaligned the robot is.
 */
public class SpeedAdjustment implements IFilter {
    
    private final IFilter adjustmentFilter;
    private final IStream angleError;

    public SpeedAdjustment(IStream angleError) {
        this.angleError = angleError;
        adjustmentFilter = new LowPassFilter(Alignment.SPEED_ADJ_FILTER);
    }

    private double getSpeedAdjustment() {
        double error = angleError.get() / Limelight.MAX_ANGLE_FOR_MOVEMENT.get();
        return adjustmentFilter.get(Math.exp(-error * error));
    }

    @Override
    public double get(double speed) {
        return getSpeedAdjustment() * speed;
    }

}
