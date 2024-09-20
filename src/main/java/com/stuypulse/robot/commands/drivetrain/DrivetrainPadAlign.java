/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import com.stuypulse.robot.commands.ThenShoot;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.util.IFuser;

import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainPadAlign extends Command {

    private final Drivetrain drivetrain;

    private final BStream finished;

    private final IFuser angleError;
    protected final Controller angleController;

    public DrivetrainPadAlign(Drivetrain drivetrain, Camera camera) {
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

        // handle errors
        this.angleController = Alignment.Angle.getController();

        // finish optimally
        finished =
                BStream.create(camera::hasTarget)
                        .and(() -> angleController.isDone(Limelight.MAX_ANGLE_ERROR.get()))
                        .filtered(new BDebounceRC.Rising(Limelight.DEBOUNCE_TIME));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setLowGear();
    }

    private double getTurn() {
        return angleController.update(angleError.get());
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(0, getTurn());
    }

    @Override
    public boolean isFinished() {
        return finished.get();
    }

    public Command thenShoot(Conveyor conveyor) {
        return new ThenShoot(this, conveyor, ConveyorMode.SEMI_AUTO);
    }
}
