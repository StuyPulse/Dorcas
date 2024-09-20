/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.constants.ShotMap;
import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.subsystems.Shooter;
import com.stuypulse.robot.util.IFuser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootAnywhere extends Command {

    private final Conveyor conveyor;
    private final Drivetrain drivetrain;
    private final Shooter shooter;

    private final IFuser angleError;
    private final IFuser distance;

    private final BStream readyToShoot;

    protected final Controller angleController;

    public ShootAnywhere(RobotContainer robot) {
        this.conveyor = robot.conveyor;
        this.drivetrain = robot.drivetrain;
        this.shooter = robot.shooter;

        // find errors
        angleError =
                new IFuser(
                        Alignment.FUSION_FILTER,
                        () ->
                                robot.camera
                                        .getXAngle()
                                        .add(
                                                Angle.fromDegrees(
                                                        ShotMap.DISTANCE_TO_YAW.interpolate(
                                                                robot.camera.getDistance())))
                                        .toDegrees(),
                        () -> drivetrain.getRawGyroAngle());

        distance =
                new IFuser(
                        Alignment.FUSION_FILTER,
                        () -> robot.camera.getDistance(),
                        () -> -drivetrain.getDistance());

        // handle errors
        this.angleController = Alignment.Angle.getController();

        // finish optimally
        readyToShoot =
                BStream.create(() -> shooter.isReady())
                        .and(() -> angleController.isDone(Limelight.MAX_ANGLE_ERROR.get()))
                        .filtered(new BDebounceRC.Rising(Limelight.DEBOUNCE_TIME));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setLowGear();
        shooter.retractHood();
    }

    private double getTurn() {
        return angleController.update(angleError.get());
    }

    private double getTargetRPM() {
        return ShotMap.DISTANCE_TO_RPM.interpolate(distance.get());
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(0.0, getTurn());
        shooter.setShooterRPM(getTargetRPM());

        if (readyToShoot.get()) {
            conveyor.setMode(ConveyorMode.SEMI_AUTO);
        } else {
            conveyor.setMode(ConveyorMode.DEFAULT);
        }

        // Debug Info
    

        SmartDashboard.putNumber("Debug/ShootAnywhere/TargetRPM", getTargetRPM());
        SmartDashboard.putNumber("Debug/ShootAnywhere/Distace", distance.get());
        SmartDashboard.putNumber("Debug/ShootAnywhere/CurrentRPM", shooter.getShooterRPM());

    }

    @Override
    public boolean isFinished() {
        return conveyor.isEmpty();
    }
}
