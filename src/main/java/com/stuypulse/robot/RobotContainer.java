
/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.*;
import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.commands.auton.*;
import com.stuypulse.robot.commands.climber.*;
import com.stuypulse.robot.commands.conveyor.*;
import com.stuypulse.robot.commands.drivetrain.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.shooter.*;
import com.stuypulse.robot.constants.*;
import com.stuypulse.robot.subsystems.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {

    // Subsystems
    public final Climber climber = new Climber();
    public final ColorSensor colorSensor = new ColorSensor();
    public final Conveyor conveyor = new Conveyor(colorSensor);
    public final Drivetrain drivetrain = new Drivetrain();
    public final Intake intake = new Intake(conveyor);
    public final LEDController leds = new LEDController(this);
    public final Pump pump = new Pump();
    public final Shooter shooter = new Shooter();

    public final Camera camera = new Camera(shooter);

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    public RobotContainer() {
        // Disable telemetry to reduce lag
        LiveWindow.disableAllTelemetry();   
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure the button bindings
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver));
        conveyor.setDefaultCommand(new ConveyorIndex(conveyor));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        /*** Climber ***/
        operator.getRightStickDown()
                .onTrue(new IntakeRetract(intake))
                .onTrue(new ShooterStop(shooter))
                .onTrue(new ShooterRetractHood(shooter))
                .whileTrue(new ConveyorStop(conveyor))
                .whileTrue(new IntakeStop(intake))
                .whileTrue(new ClimberMoveUp(climber));

        operator.getRightStickUp()
                .onTrue(new IntakeRetract(intake))
                .onTrue(new ShooterStop(shooter))
                .onTrue(new ShooterRetractHood(shooter))
                .whileTrue(new ConveyorStop(conveyor))
                .whileTrue(new IntakeStop(intake))
                .whileTrue(new DrivetrainStop(drivetrain))
                .whileTrue(new ClimberMoveDown(climber));

        operator.getLeftStickRight().onTrue(new ClimberMaxTilt(climber));
        operator.getLeftStickLeft().onTrue(new ClimberNoTilt(climber));

        operator.getRightMenuButton().whileTrue(new ClimberForceLower(climber));

        /*** Conveyor ***/
        operator.getTopButton().whileTrue(new ConveyorStop(conveyor));
        operator.getLeftButton().whileTrue(new ConveyorForceEject(conveyor));

        /*** Drivetrain ***/

        // Right Button --> Low Gear

        // driver.getLeftButton() // --> Low Gear
        //         .whileTrue(new ShooterFenderShot(shooter))
        //         .whileTrue(new WaitCommand(0.3).andThen(new ConveyorShootSemi(conveyor).perpetually())); 

        driver.getBottomButton()
                .whileTrue(new ShooterRingShot(shooter))
                .whileTrue(new DrivetrainAlign(drivetrain, camera).thenShoot(conveyor));

        driver.getLeftBumper()
                .whileTrue(new ShooterPadShot(shooter))
                .whileTrue(new DrivetrainPadAlign(drivetrain, camera).thenShoot(conveyor));

        // driver.getRightButton().whileTrue(new SimpleAlignShoot(this));
        
        driver.getTopButton().whileTrue(new BetterShootAnywhere(this).repeatedly());
        // driver.getTopButton().whileTrue(new TestAlign(this).thenShoot(conveyor));

        driver.getRightBumper().whileTrue(new DrivetrainAlign(drivetrain, camera).repeatedly());

        /*** Intake ***/
        operator.getRightTriggerButton()
                .onTrue(new IntakeExtend(intake))
                .whileTrue(new IntakeAcquire(intake))
                .onFalse(
                        new IntakeRetract(intake));

        operator.getRightBumper()
                .onTrue(new IntakeExtend(intake))
                .whileTrue(new IntakeAcquire(intake))
                .whileTrue(new ConveyorForceIntake(conveyor))
                .onFalse(new IntakeRetract(intake));

        operator.getLeftTriggerButton().whileTrue(new IntakeDeacquire(intake));

        operator.getDPadUp().onTrue(new IntakeRetract(intake));

        /*** Shooter ***/
        operator.getDPadLeft().onTrue(new ShooterFenderShot(shooter));
        operator.getDPadRight().onTrue(new ShooterRingShot(shooter));
        operator.getDPadDown().onTrue(new ShooterPadShot(shooter));
        operator.getDPadUp().onTrue(new ClimberJiggle(climber));

        operator.getRightButton().whileTrue(new ConveyorShoot(conveyor).repeatedly());

        operator.getLeftBumper().onTrue(new ShooterStop(shooter));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.addOption("0 Ball", new MobilityAuton.NoEncoders(this));
        autonChooser.addOption("2 Ball", new TwoBallAuton(this));
        autonChooser.addOption("2 Ball Sam Mean", new TwoBallMeanerAuton(this));

        autonChooser.setDefaultOption("5 Ball [DEFAULT]", new FiveBallAuton(this));
        autonChooser.addOption("Partner Ball", new PartnerBallAuton(this));
        autonChooser.addOption("Two Ball One Mean", new TwoBallOneMeanAuton(this));
        autonChooser.addOption("Four Ball", new FourBallAuton(this));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}

