/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {

    Config CLIMBER = new Config(true, IdleMode.kBrake, 80, 1.0 / 5.0);

    public interface Conveyor {
        Config GANDALF = new Config(true, IdleMode.kBrake, 40);
        Config TOP_BELT = new Config(false, IdleMode.kBrake, 30);
    }

    public interface Drivetrain {
        int CURRENT_LIMIT_AMPS = 60;
        IdleMode IDLE_MODE = IdleMode.kBrake;

        Config LEFT = new Config(true, IDLE_MODE, CURRENT_LIMIT_AMPS);
        Config RIGHT = new Config(false, IDLE_MODE, CURRENT_LIMIT_AMPS);
    }

    Config INTAKE = new Config(true, IdleMode.kBrake, 50);

    public interface Shooter {
        Config LEFT = new Config(false, IdleMode.kCoast, 60);
        Config RIGHT = new Config(true, IdleMode.kCoast, 60);
        Config FEEDER = new Config(false, IdleMode.kCoast, 80);
    }

    /** Class to store all of the values a motor needs */
    public static class Config {
        public final boolean INVERTED;
        public final IdleMode IDLE_MODE;
        public final int CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public Config(
                boolean inverted,
                IdleMode idleMode,
                int currentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.IDLE_MODE = idleMode;
            this.CURRENT_LIMIT_AMPS = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public Config(boolean inverted, IdleMode idleMode, int currentLimitAmps) {
            this(inverted, idleMode, currentLimitAmps, 0.0);
        }

        public Config(boolean inverted, IdleMode idleMode) {
            this(inverted, idleMode, 80);
        }

        public void configure(CANSparkMax motor) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.burnFlash();
        }
    }
}
