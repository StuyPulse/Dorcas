/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.streams.IFuser;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import com.stuypulse.robot.commands.ThenShoot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainAlign extends CommandBase {

    private final Drivetrain drivetrain;
    private final Camera camera;

    private final Debouncer finished;

    private IFilter speedAdjFilter;

    private final IFuser angleError;
    private final IFuser distanceError;

    protected final Controller angleController;
    protected final Controller distanceController;

    public DrivetrainAlign(Drivetrain drivetrain, Camera camera) {
        this.drivetrain = drivetrain;
        this.camera = camera;

        // find errors
        angleError =
                new IFuser(
                        Alignment.FUSION_FILTER,
                        () -> camera.getXAngle().toDegrees(),
                        () -> drivetrain.getRawGyroAngle());

        distanceError =
                new IFuser(
                        Alignment.FUSION_FILTER,
                        () -> Settings.Limelight.RING_SHOT_DISTANCE - camera.getDistance(),
                        () -> drivetrain.getDistance());

        // handle errors
        speedAdjFilter = new LowPassFilter(Alignment.SPEED_ADJ_FILTER);
        this.angleController = Alignment.Angle.getController();
        this.distanceController = Alignment.Speed.getController();

        // finish optimally
        finished = new Debouncer(Limelight.DEBOUNCE_TIME, DebounceType.kRising);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setLowGear();

        speedAdjFilter = new LowPassFilter(Alignment.SPEED_ADJ_FILTER);

        angleError.initialize();
        distanceError.initialize();
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
        return finished.calculate(
                camera.hasTarget()
                        && drivetrain.getVelocity() < Limelight.MAX_VELOCITY.get()
                        && angleController.isDone(Limelight.MAX_ANGLE_ERROR.get())
                        && distanceController.isDone(Limelight.MAX_DISTANCE_ERROR.get()));
    }

    public Command thenShoot(Conveyor conveyor) {
        return new ThenShoot(this, conveyor);
    }
}