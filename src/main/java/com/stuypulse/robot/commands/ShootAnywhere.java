/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.math.interpolation.CubicInterpolator;
import com.stuypulse.stuylib.math.interpolation.Interpolator;
import com.stuypulse.stuylib.math.interpolation.NearestInterpolator;
import com.stuypulse.stuylib.streams.IFuser;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.conveyor.modes.ConveyorMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootAnywhere extends CommandBase {

    private final Conveyor conveyor;
    private final Drivetrain drivetrain;
    private final Shooter shooter;

    private final IFuser angleError;
    private final IFuser distance;

    private final BStream readyToShoot;

    protected final Controller angleController;

    private final Interpolator distanceToRPM = new NearestInterpolator(
        new Vector2D(Limelight.RING_DISTANCE.get(), Settings.Shooter.RING_RPM.get()),
        new Vector2D(Limelight.PAD_DISTANCE.get(), Settings.Shooter.PAD_RPM.get())
    );

    public ShootAnywhere(RobotContainer robot) {
        this.conveyor = robot.conveyor;
        this.drivetrain = robot.drivetrain;
        this.shooter = robot.shooter;

        // find errors
        angleError =
                new IFuser(
                        Alignment.FUSION_FILTER,
                        () -> robot.camera.getXAngle()
                                .add(Angle.fromDegrees(Limelight.RING_YAW.get()))
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
        angleError.initialize();
        distance.initialize();
    }

    private double getTurn() {
        return angleController.update(angleError.get());
    }

    private double getTargetRPM() {
        return distanceToRPM.get(distance.get());
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(0.0, getTurn());
        shooter.setShooterRPM(getTargetRPM());

        if(readyToShoot.get()) {
            conveyor.setMode(ConveyorMode.SHOOT);
        } else {
            conveyor.setMode(ConveyorMode.DEFAULT);
        }

        // Debug Info
        if (Settings.DEBUG_MODE.get()) {

            SmartDashboard.putNumber("Debug/ShootAnywhere/TargetRPM", getTargetRPM());
            SmartDashboard.putNumber("Debug/ShootAnywhere/Distace", distance.get());
            SmartDashboard.putNumber("Debug/ShootAnywhere/CurrentRPM", shooter.getShooterRPM());

        }
        
    }

    @Override
    public boolean isFinished() {
        return conveyor.isEmpty();
    }
}
