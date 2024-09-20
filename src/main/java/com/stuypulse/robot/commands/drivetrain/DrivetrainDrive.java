/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.Command;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Drivetrain.Stalling;
import com.stuypulse.robot.subsystems.Drivetrain;

public class DrivetrainDrive extends Command {

    private final Drivetrain drivetrain;
    private final Gamepad driver;

    private final BStream stalling;
    private final IStream speed, angle;

    public DrivetrainDrive(Drivetrain drivetrain, Gamepad driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;

        this.stalling =
                Stalling.STALL_DETECTION
                        .and(drivetrain::isStalling)
                        .filtered(new BDebounceRC.Both(Settings.Drivetrain.Stalling.DEBOUNCE_TIME));

        this.speed =
                IStream.create(() -> driver.getRightTrigger() - driver.getLeftTrigger())
                        .filtered(
                                x -> SLMath.deadband(x, Settings.Drivetrain.SPEED_DEADBAND.get()),
                                x -> SLMath.spow(x, Settings.Drivetrain.SPEED_POWER.get()),
                                new LowPassFilter(Settings.Drivetrain.SPEED_FILTER));

        this.angle =
                IStream.create(() -> driver.getLeftX())
                        .filtered(
                                x -> SLMath.deadband(x, Settings.Drivetrain.ANGLE_DEADBAND.get()),
                                x -> SLMath.spow(x, Settings.Drivetrain.ANGLE_POWER.get()),
                                new LowPassFilter(Settings.Drivetrain.ANGLE_FILTER));

        addRequirements(drivetrain);
    }

    public void execute() {
        if (driver.getRawLeftButton()) {
            drivetrain.setLowGear();
            drivetrain.arcadeDrive(speed.get() - 0.1, angle.get());
        } else if (driver.getRawRightButton() || stalling.get()) {
            drivetrain.setLowGear();
            drivetrain.arcadeDrive(speed.get(), angle.get());
        } else {
            drivetrain.setHighGear();
            drivetrain.curvatureDrive(speed.get(), angle.get());
        }
    }

    public boolean isFinished() {
        return false;
    }
}
