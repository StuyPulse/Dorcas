/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Climber.Stalling;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

/*-
 * Climbs at end of match
 *
 * Contains:
 *      - Change tilt of climber
 *      - Move climber via motor
 *      - Different tilt angles
 *      - Encoder + Solenoid used for stopping
 *
 * @author independence106(Jason Zhou)
 * @author Ca7Ac1(Ayan Chowdhury)
 * @author marcjiang7(Marc Jiang)
 * @author ambers7(Amber Shen)
 * @author Souloutz(Howard Kong)
 * @author jiayuyan0501(Jiayu Yan)
 * @author ijiang05(Ian Jiang)
 * @author TraceyLin(Tracey Lin)
 * @author annazheng14(Anna Zheng)
 * @author lonelydot(Raymond Zhang)
 * @author andylin2004(Andy Lin)
 * @author hwang30git(Hui Wang)
 */
public class Climber extends SubsystemBase {

    public enum Tilt {
        MAX_TILT(Value.kReverse),
        NO_TILT(Value.kForward);

        private final Value extended;

        private Tilt(Value extended) {
            this.extended = extended;
        }
    }

    private final CANSparkMax climber;
    private final RelativeEncoder encoder;

    private final Debouncer stalling;

    private final Solenoid stopper;

    private final DoubleSolenoid tilter;

    public Climber() {
        climber = new CANSparkMax(Ports.Climber.MOTOR, MotorType.kBrushless);

        encoder = climber.getEncoder();
        encoder.setPositionConversionFactor(Settings.Climber.ENCODER_RATIO);
        encoder.setVelocityConversionFactor(Settings.Climber.ENCODER_RATIO / 60.0);

        Motors.CLIMBER.configure(climber);

        stalling = new Debouncer(Stalling.DEBOUNCE_TIME, DebounceType.kBoth);   

        stopper = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Climber.STOPPER);
        tilter =
                new DoubleSolenoid(
                        PneumaticsModuleType.CTREPCM,
                        Ports.Climber.TILTER_FORWARD,
                        Ports.Climber.TILTER_REVERSE);
    }

    /*** MOTOR CONTROL ***/

    public void forceLowerClimber() {
        if (getLocked()) {
            Settings.reportWarning("Climber attempted to run while lock was enabled!");
            setMotorStop();
        } else {
            climber.set(-Settings.Climber.SLOW_SPEED.get());
            resetEncoder();
        }
    }

    public void setMotor(double speed) {
        if (speed != 0.0 && isStalling()) {
            DriverStation.reportError(
                    "[CRITICAL] Climber is stalling when attempting to move!", false);
            stalling.calculate(true);
            setMotorStop();
        } else if (speed != 0.0 && getLocked()) {
            Settings.reportWarning("Climber attempted to run while lock was enabled!");
            setMotorStop();
        } else if (speed > 0.0 && getTopHeightLimitReached()) {
            Settings.reportWarning("Climber attempted to run past top height limit!");
            setMotorStop();
        } else if (speed < 0.0 && getBottomHeightLimitReached()) {
            Settings.reportWarning("Climber attempted to run past bottom height limit!");
            setMotorStop();
        } else {
            climber.set(speed);
        }
    }

    public void setMotorStop() {
        climber.stopMotor();
    }

    /*** BRAKE CONTROL ***/

    public void setLocked() {
        setMotorStop();
        stopper.set(false);
    }

    public void setUnlocked() {
        stopper.set(true);
    }

    public boolean getLocked() {
        return stopper.get();
    }

    /*** TILT CONTROL ***/

    public void setTilt(Tilt tilt) {
        tilter.set(tilt.extended);
    }

    /*** ENCODER ***/

    public double getPosition() {
        return encoder.getPosition();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public boolean getTopHeightLimitReached() {
        return Settings.Climber.ENABLE_ENCODERS.get()
                && getPosition() >= Settings.Climber.MAX_EXTENSION.get();
    }

    public boolean getBottomHeightLimitReached() {
        return Settings.Climber.ENABLE_ENCODERS.get() && getPosition() <= 0;
    }

    /*** STALL PROTECTION ***/

    private double getDutyCycle() {
        return climber.get();
    }

    private double getCurrentAmps() {
        return Math.abs(climber.getOutputCurrent());
    }

    private double getVelocity() {
        return encoder.getVelocity();
    }

    public boolean isStalling() {
        boolean current = getCurrentAmps() > Stalling.CURRENT_THRESHOLD;
        boolean output = Math.abs(getDutyCycle()) > Stalling.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(getVelocity()) < Stalling.VELOCITY_THESHOLD;
        return stalling.calculate((current || output) && velocity);
    }

    /*** DEBUG INFORMATION ***/

    @Override
    public void periodic() {
        if (isStalling()) {
            DriverStation.reportError(
                    "[CRITICAL] Climber is stalling when attempting to move!", false);
            setMotorStop();
        }

        // This method will be called once per scheduler run
        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putBoolean("Debug/Climber/Stalling", isStalling());
            SmartDashboard.putNumber("Debug/Climber/Current Amps", getCurrentAmps());
            SmartDashboard.putNumber("Debug/Climber/Velocity", getVelocity());

            SmartDashboard.putBoolean(
                    "Debug/Climber/Max Tilt", tilter.get().equals(Value.kReverse));
            SmartDashboard.putBoolean("Debug/Climber/Stopper Active", stopper.get());
            SmartDashboard.putNumber("Debug/Climber/Climber Speed", climber.get());
        }
    }
}
