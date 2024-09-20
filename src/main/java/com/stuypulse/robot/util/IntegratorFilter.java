/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;

/**
 * A standard integrator filter for a PID controllers.
 *
 * <p>Implements integral range and integral limits.
 *
 * @author Myles Pasetsky (@selym3)
 */
public class IntegratorFilter implements IFilter {

    /** controller that this integral filter acts on */
    private final PIDController pidController;

    /**
     * under which error should error begin to accumulate
     *
     * <p>(e.g. handle steady state only when near setpoint)
     */
    private final Number range;

    /** the max value the integrated error can reach */
    private final Number limit;

    public IntegratorFilter(PIDController pidController, Number range, Number limit) {
        this.pidController = pidController;
        this.range = range;
        this.limit = limit;
    }

    /**
     * given the integrated error so far (which has also been calculated by this filter), returns
     * the next value of the integrated error
     */
    @Override
    public double get(double next) {
        if (pidController.isDone(range.doubleValue())) {
            return SLMath.clamp(next, limit.doubleValue());
        } else {
            return 0.0;
        }
    }
}
