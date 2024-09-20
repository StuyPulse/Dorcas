/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.conveyor.ConveyorShoot;
import com.stuypulse.robot.commands.drivetrain.DrivetrainAlign;
import com.stuypulse.robot.commands.drivetrain.DrivetrainRamsete;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.shooter.ShooterRingShot;
import com.stuypulse.robot.util.LEDColor;

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

public class FourBallAuton extends SequentialCommandGroup {
    // Time it takes for the intake to go down
    private static final double INTAKE_FALL_DOWN = 0.2;
    // Time it takes for the shooter to reach the target speed
    private static final double SHOOTER_INITIALIZE_DELAY = 1.0;
    // Time it takes for the conveyor to give the shooter the ball
    private static final double CONVEYOR_TO_SHOOTER = 2.0;
    // Time we want to give the drivetrain to align
    private static final double DRIVETRAIN_ALIGN_TIME = 3.0;
    // Time it takes for human player to roll ball to intake
    private static final double HUMAN_WAIT_TIME = 1.5;

    private static final String FOUR_BALL_START =
            "FourBallAuton/output/FourBallAutonGetSecondBall.wpilib.json";
    private static final String FOUR_BALL_TO_TERMINAL =
            "FourBallAuton/output/FourBallAutonGetTerminalBalls.wpilib.json";
    private static final String FOUR_BALL_SHOOT_TERMINAL_BALLS =
            "FourBallAuton/output/FourBallAutonShootTerminalBalls.wpilib.json";

    /** Creates a new FourBallAuton. */
    public FourBallAuton(RobotContainer robot) {

        // Starting up subsystems
        addCommands(
            new LEDSet(robot.leds, LEDColor.YELLOW.pulse()),

            new ShooterRingShot(robot.shooter),
            new IntakeExtend(robot.intake),
            new WaitCommand(INTAKE_FALL_DOWN),
            new IntakeAcquireForever(robot.intake),
            new WaitCommand(SHOOTER_INITIALIZE_DELAY)
        );

        // Tarmac to first ball
        addCommands(
            new LEDSet(robot.leds, LEDColor.GREEN),
            new DrivetrainRamsete(robot.drivetrain, FOUR_BALL_START).robotRelative());

        addCommands(
            new LEDSet(robot.leds, LEDColor.GREEN.pulse()),
            new DrivetrainAlign(robot.drivetrain, robot.camera)
                .withTimeout(DRIVETRAIN_ALIGN_TIME));
        addCommands(
            new LEDSet(robot.leds, LEDColor.RAINBOW),
            new ConveyorShoot(robot.conveyor).withTimeout(CONVEYOR_TO_SHOOTER)
        );

        // First ball to terminal to RingShot
        addCommands(
            new LEDSet(robot.leds, LEDColor.BLUE),

            new DrivetrainRamsete(robot.drivetrain, FOUR_BALL_TO_TERMINAL)
                .fieldRelative()
        );

        addCommands(
            new LEDSet(robot.leds, LEDColor.WHITE.pulse()),

            new WaitCommand(HUMAN_WAIT_TIME).until(() -> robot.conveyor.isFull())
        );

        addCommands(
            new LEDSet(robot.leds, LEDColor.PURPLE),

            new DrivetrainRamsete(robot.drivetrain, FOUR_BALL_SHOOT_TERMINAL_BALLS)
                    .fieldRelative());
    
        addCommands(
            new LEDSet(robot.leds, LEDColor.PURPLE.pulse()),
            new DrivetrainAlign(robot.drivetrain, robot.camera)
                    .withTimeout(DRIVETRAIN_ALIGN_TIME)
        );

        addCommands(
            new LEDSet(robot.leds, LEDColor.RAINBOW),
            new ConveyorShoot(robot.conveyor).repeatedly()
        );

    }
}
