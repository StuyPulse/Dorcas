/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.conveyor.ConveyorShootCommand;
import com.stuypulse.robot.commands.drivetrain.DrivetrainAlignCommand;
import com.stuypulse.robot.commands.drivetrain.DrivetrainDriveDistanceCommand;
import com.stuypulse.robot.commands.drivetrain.DrivetrainRamseteCommand;
import com.stuypulse.robot.commands.intake.IntakeAcquireForeverCommand;
import com.stuypulse.robot.commands.intake.IntakeExtendCommand;
import com.stuypulse.robot.commands.leds.LEDSetCommand;
import com.stuypulse.robot.commands.shooter.ShooterRingShotCommand;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/*-
 * @author Vincent Wang (vinowang921@gmail.com)
 * @author Ivan Wei (ivanw8288@gmail.com)
 * @author Ivan Chen (ivanchen07@gmail.com)
 * @author Eric Lin (ericlin071906@gmail.com)
 * @author Marc Jiang (mjiang05@gmail.com)
 * @author Ian Jiang (ijiang05@gmail.com)
 * @author Carmin Vuong (carminvuong@gmail.com)
 * @author Samuel Chen(samchen1738@gmail.com)
 */
 
public class FiveBallBlueAuton extends SequentialCommandGroup {
    // Time it takes for the shooter to reach the target speed
    private static final double SHOOTER_INITIALIZE_DELAY = 0.3;
    // Time it takes for the conveyor to give the shooter the ball
    private static final double CONVEYOR_TO_SHOOTER = 3.0;
    // Time we want to give the drivetrain to align
    private static final double DRIVETRAIN_ALIGN_TIME = 3.0;

    private static final double HUMAN_WAIT_TIME = 0;

    private static final String FIVE_BALL_TO_SECOND_BALL = "FiveBallBlueAuton/output/FiveBallAcquireSecondBall.wpilib.json";
    private static final String FIVE_BALL_TO_TERMINAL = "FiveBallBlueAuton/output/FiveBallGetTerminalBalls.wpilib.json";
    private static final String FIVE_BALL_TERMINAL_TO_SHOOT = "FiveBallBlueAuton/output/FiveBallShootTerminalBalls.wpilib.json";
    private static final String FIVE_BALL_TO_WALL_BALL = "FiveBallBlueAuton/output/FiveBallGetWallBall.wpilib.json";

    /** Creates a new FiveBallAuton. */
    public FiveBallBlueAuton(RobotContainer robot) {

        // Starting up subsystems
        addCommands(
            new LEDSetCommand(robot.leds, LEDColor.YELLOW),
            new IntakeExtendCommand(robot.intake),
            new IntakeAcquireForeverCommand(robot.intake),
            new ShooterRingShotCommand(robot.shooter),
            new WaitCommand(SHOOTER_INITIALIZE_DELAY)
        );

        // Tarmac to first ball
        addCommands(
            new LEDSetCommand(robot.leds, LEDColor.GREEN),
            new DrivetrainRamseteCommand(robot.drivetrain, FIVE_BALL_TO_SECOND_BALL)
                    .robotRelative());
        addCommands(
        );
        addCommands(
            new LEDSetCommand(robot.leds, LEDColor.GREEN.pulse()),
            new DrivetrainAlignCommand(robot.drivetrain, Limelight.RING_SHOT_DISTANCE)
                    .withTimeout(DRIVETRAIN_ALIGN_TIME)
        );

        addCommands(
                new LEDSetCommand(robot.leds, LEDColor.RAINBOW),
                new ConveyorShootCommand(robot.conveyor).withTimeout(CONVEYOR_TO_SHOOTER));

        // First ball to terminal to RingShot
        addCommands(
                new LEDSetCommand(robot.leds, LEDColor.BLUE),
                new DrivetrainRamseteCommand(robot.drivetrain, FIVE_BALL_TO_TERMINAL)
                        .fieldRelative());

        addCommands(
                new DrivetrainDriveDistanceCommand(robot.drivetrain, Units.feetToMeters(-1))
                .fieldRelative(),

                new LEDSetCommand(robot.leds, LEDColor.WHITE.pulse()),

                // new WaitCommand(HUMAN_WAIT_TIME).withInterrupt(() -> robot.conveyor.isFull()),
                new WaitCommand(HUMAN_WAIT_TIME).withInterrupt(() -> robot.conveyor.isFull()));

        // Return to Ring to shoot
        addCommands(
                new LEDSetCommand(robot.leds, LEDColor.PURPLE),
                new DrivetrainRamseteCommand(robot.drivetrain, FIVE_BALL_TERMINAL_TO_SHOOT)
                        .fieldRelative());
        addCommands(
                new LEDSetCommand(robot.leds, LEDColor.PURPLE.pulse()),
                new DrivetrainAlignCommand(robot.drivetrain, Limelight.RING_SHOT_DISTANCE)
                        .withTimeout(DRIVETRAIN_ALIGN_TIME)
        );

        addCommands(
                new LEDSetCommand(robot.leds, LEDColor.RAINBOW),
                new ConveyorShootCommand(robot.conveyor).withTimeout(CONVEYOR_TO_SHOOTER));

        // Pick up and shoot fifth ball
        addCommands(
                new LEDSetCommand(robot.leds, LEDColor.PINK),
                new DrivetrainRamseteCommand(robot.drivetrain, FIVE_BALL_TO_WALL_BALL)
                        .fieldRelative());
        addCommands(  
                new LEDSetCommand(robot.leds, LEDColor.PINK.pulse()),
                new DrivetrainAlignCommand(robot.drivetrain, Limelight.RING_SHOT_DISTANCE)
                        .withTimeout(DRIVETRAIN_ALIGN_TIME)
        );

        addCommands(
                new LEDSetCommand(robot.leds, LEDColor.RAINBOW),
                new ConveyorShootCommand(robot.conveyor).withTimeout(CONVEYOR_TO_SHOOTER));

        addCommands(new LEDSetCommand(robot.leds, LEDColor.WHITE.pulse()));
    }
}