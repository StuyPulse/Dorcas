/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.filters.RateLimit;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Shooter subsystem for shooting balls out of the robot.
 *
 * <p>Uses feedforward to control two flywheels -- one for shooting and one for feeding. This drives
 * the wheels to a desired RPM.
 *
 * <p>Feedforward models (and feedback gains) are obtained through system identification.
 *
 * <p>Also contains an adjustable hood, which physically allows for two shooting angles.
 *
 * @author Myles Pasetsky (@selym3)
 */
public class Shooter extends SubsystemBase {

    private final SmartNumber targetRPM;
    private final IFilter targetFilter;

    private final PIDFlywheel shooter;
    private final PIDFlywheel feeder;

    private final Solenoid hood;

    public Shooter() {
        /** TARGET RPM VARIABLES * */
        targetRPM = new SmartNumber("Shooter/Target RPM", 0.0);
        targetFilter =
                new RateLimit(Settings.Shooter.MAX_TARGET_RPM_CHANGE)
                        .then(new LowPassFilter(Settings.Shooter.CHANGE_RC));

        /** SHOOTER * */
        CANSparkMax shooterMotor = new CANSparkMax(Ports.Shooter.LEFT, MotorType.kBrushless);
        CANSparkMax shooterFollower = new CANSparkMax(Ports.Shooter.RIGHT, MotorType.kBrushless);

        shooter =
                new PIDFlywheel(
                        shooterMotor,
                        Settings.Shooter.ShooterFF.getController(),
                        Settings.Shooter.ShooterPID.getController());
        shooter.addFollower(shooterFollower);

        /** FEEDER * */
        CANSparkMax feederMotor = new CANSparkMax(Ports.Shooter.FEEDER, MotorType.kBrushless);

        feeder =
                new PIDFlywheel(
                        feederMotor,
                        Settings.Shooter.FeederFF.getController(),
                        Settings.Shooter.FeederPID.getController());

        /** HOOD * */
        hood = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Shooter.HOOD_SOLENOID);

        /** CONFIG MOTORS */
        Motors.Shooter.LEFT.configure(shooterMotor);
        Motors.Shooter.RIGHT.configure(shooterFollower);
        Motors.Shooter.FEEDER.configure(feederMotor);
    }

    /*** SHOOTER CONTROL ***/

    public void setShooterRPM(Number speed) {
        targetRPM.set(speed);
    }

    public void extendHood() {
        hood.set(true);
    }

    public void retractHood() {
        hood.set(false);
    }

    /*** ENCODER READINGS ***/

    public double getShooterRPM() {
        return shooter.getVelocity();
    }

    public double getFeederRPM() {
        return feeder.getVelocity();
    }

    public boolean isFenderMode() {
        return hood.get();
    }

    /*** TARGET RPM READING ***/

    public double getRawTargetRPM() {
        return targetRPM.get();
    }

    public double getTargetRPM() {
        return targetFilter.get(getRawTargetRPM());
    }

    public boolean isReady() {
        return Math.abs(getShooterRPM() - getRawTargetRPM()) < Settings.Shooter.MAX_RPM_ERROR;
    }

    @Override
    public void periodic() {
        double setpoint = getTargetRPM();

        if (setpoint < Settings.Shooter.MIN_RPM) {
            shooter.stop();
            feeder.stop();
        } else {
            shooter.setVelocity(setpoint);
            feeder.setVelocity(setpoint * Settings.Shooter.FEEDER_MULTIPLER.get());
        }

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("Debug/Shooter/Shooter RPM", getShooterRPM());
            SmartDashboard.putNumber("Debug/Shooter/Feeder RPM", getFeederRPM());
        }
    }
}
