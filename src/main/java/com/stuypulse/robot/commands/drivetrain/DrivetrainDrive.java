/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.filters.JerkLimit;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Drivetrain.Stalling;
import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDrive extends CommandBase {

    private static final double kMaxVelocity = 4.0; // m/s
    private static final double kMaxAcceleration = 6.0; // m/s/s
    private static final double kMaxJerk = 200.0; // m/s/s/s

    private static final double kMaxAngularVelocity = 3.0 * Math.PI; // rad/s
    private static final double kMaxAngularAcceleration = 30.0 * Math.PI; // rad/s/s
    // Angular Jerk is stupid and doesnt exist

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
                                x -> x * kMaxVelocity,
                                new JerkLimit(kMaxAcceleration, kMaxJerk));

        this.angle =
                IStream.create(() -> driver.getLeftX())
                        .filtered(
                                x -> SLMath.deadband(x, Settings.Drivetrain.ANGLE_DEADBAND.get()),
                                x -> x * kMaxAngularVelocity,
                                new JerkLimit(kMaxAngularAcceleration, -1));

        addRequirements(drivetrain);
    }

    public void execute() {
        if (driver.getRawRightButton() || stalling.get()) {
            drivetrain.setLowGear();
        } else {
            drivetrain.setHighGear();
        }

        drivetrain.arcadeDriveUnits(speed.get(), angle.get(), kMaxVelocity);
    }

    public boolean isFinished() {
        return false;
    }
}
