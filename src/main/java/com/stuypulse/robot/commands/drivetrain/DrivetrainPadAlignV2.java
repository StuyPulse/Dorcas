/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.robot.commands.ThenShoot;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.util.IFuser;

import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainPadAlignV2 extends Command {

    private final Drivetrain drivetrain;

    private final BStream finished;

    private IFilter speedAdjFilter;

    private final IFuser angleError;
    private final IFuser distanceError;

    protected final Controller angleController;
    protected final Controller distanceController;

    public DrivetrainPadAlignV2(Drivetrain drivetrain, Camera camera) {
        this.drivetrain = drivetrain;

        // find errors
        angleError =
                new IFuser(
                        Alignment.FUSION_FILTER,
                        () ->
                                camera.getXAngle()
                                        .add(Angle.fromDegrees(Limelight.PAD_YAW.get()))
                                        .toDegrees(),
                        () -> drivetrain.getRawGyroAngle());

        distanceError =
                new IFuser(
                        Alignment.FUSION_FILTER,
                        () -> Settings.Limelight.PAD_DISTANCE.get() - camera.getDistance(),
                        () -> drivetrain.getDistance());

        // handle errors
        speedAdjFilter = new LowPassFilter(Alignment.SPEED_ADJ_FILTER);
        this.angleController = Alignment.Angle.getController();
        this.distanceController = Alignment.Speed.getController();

        // finish optimally
        finished =
                BStream.create(camera::hasTarget)
                        .and(
                                () ->
                                        Math.abs(drivetrain.getVelocity())
                                                < Limelight.MAX_VELOCITY.get())
                        .and(() -> angleController.isDone(Limelight.MAX_ANGLE_ERROR.get()))
                        .and(() -> distanceController.isDone(Limelight.MAX_DISTANCE_ERROR.get()))
                        .filtered(new BDebounceRC.Rising(Limelight.DEBOUNCE_TIME));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setLowGear();

        speedAdjFilter = new LowPassFilter(Alignment.SPEED_ADJ_FILTER);
    }

    private double getSpeedAdjustment() {
        double error = angleError.get() / Limelight.MAX_ANGLE_FOR_MOVEMENT.get();
        return speedAdjFilter.get(Math.exp(-error * error));
    }

    private double getSpeed() {
        double speed = distanceController.update(distanceError.get());
        return speed * getSpeedAdjustment();
    }

    private double getTurn() {
        return angleController.update(angleError.get());
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(getSpeed(), getTurn());
    }

    @Override
    public boolean isFinished() {
        return finished.get();
    }

    public Command thenShoot(Conveyor conveyor) {
        return new ThenShoot(this, conveyor, ConveyorMode.SEMI_AUTO);
    }
}
