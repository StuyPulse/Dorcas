/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.util.Target;

import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * @author Ivan Wei (ivanw8288@gmail.com)
 * 
 * A command group that allows direct switch from path following into target alignment
 * to avoid the extra period of time of alignment after completing a path.
 */
public class DrivetrainHybridRamseteAlignCommand extends SequentialCommandGroup {

    /**
     * Creates a Hybrid Ramsete Align command which immediately swaps from path following
     * to alignment as soon as a target is acquired.
     * @param alignmentTimeout time allowed for alignment
     */
    public DrivetrainHybridRamseteAlignCommand(DrivetrainRamseteCommand ramseteCommand, DrivetrainAlignCommand alignCommand, double alignmentTimeout) {
        addCommands(
            ramseteCommand.withInterrupt(() -> Target.hasTarget()),
            alignCommand.withTimeout(alignmentTimeout)
        );
    }

    /**
     * Creates a Hybrid Ramsete Align command which swaps from path following to alignment when
     * a target is acquired AND the robot is below a certain distance away.
     * @param alignmentTimeout time allowed for alignment
     * @param swapDistance distance error at which below to switch to alignment
     */
    public DrivetrainHybridRamseteAlignCommand(DrivetrainRamseteCommand ramseteCommand, DrivetrainAlignCommand alignCommand, double alignmentTimeout, double swapDistance) {
        addCommands(
            ramseteCommand.withInterrupt(() -> Target.hasTarget() && (Target.getDistance() < swapDistance)),
            alignCommand.withTimeout(alignmentTimeout)
        );
    }

    @Override
    public void initialize() {
        // Although AlignCommand will enable the limelight LED, we want it on before to check 
        // if a target enters range to run AlignCommand in the first place
        Target.enable();
    }

    @Override
    public void end(boolean interrupted) {
        Target.disable();
    }
}
