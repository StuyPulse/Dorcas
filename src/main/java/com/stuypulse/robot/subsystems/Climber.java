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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
    // private final RelativeEncoder encoder;

    private final Debouncer stalling;

    private final DoubleSolenoid tilter;

    private final DigitalInput left;
    private final DigitalInput right;

    public Climber() {
        climber = new CANSparkMax(Ports.Climber.MOTOR, MotorType.kBrushless);

        // encoder = climber.getEncoder();
        // encoder.setPositionConversionFactor(Encoders.ENCODER_RATIO);
        // encoder.setVelocityConversionFactor(Encoders.ENCODER_RATIO / 60.0);

        Motors.CLIMBER.configure(climber);

        stalling = new Debouncer(Stalling.DEBOUNCE_TIME, DebounceType.kBoth);

        tilter =
                new DoubleSolenoid(
                        PneumaticsModuleType.CTREPCM,
                        Ports.Climber.TILTER_FORWARD,
                        Ports.Climber.TILTER_REVERSE);

        left = new DigitalInput(Ports.Climber.LEFT_LIMIT);
        right = new DigitalInput(Ports.Climber.RIGHT_LIMIT);
    }

    /*** MOTOR CONTROL ***/

    public void forceLowerClimber() {
        climber.set(-Settings.Climber.SLOW_SPEED.get());
        resetEncoder();
    }

    public void setMotor(double speed) {
        if (speed != 0.0 && isStalling()) {
            DriverStation.reportError(
                    "[CRITICAL] Climber is stalling when attempting to move!", false);
            stalling.calculate(true);
            setMotorStop();
        } else if (speed < 0.0 && getHookClear()) {
            Settings.reportWarning("Climber attempted to run past bottom limit!");
            setMotorStop();
        } else {
            climber.set(speed);
        }
    }

    public void setMotorStop() {
        climber.stopMotor();
    }

    /*** TILT CONTROL ***/

    public void setTilt(Tilt tilt) {
        tilter.set(tilt.extended);
    }

    /*** ENCODER ***/

    public double getVelocity() {
        return 0.0; // encoder.getVelocity();
    }

    public double getPosition() {
        return 0.0; // encoder.getPosition();
    }

    public void resetEncoder() {
        // encoder.setPosition(0);
    }

    public boolean getTopHeightLimitReached() {
        return false; // Encoders.ENABLED.get() && getPosition() >= Encoders.MAX_EXTENSION.get();
    }

    public boolean getBottomHeightLimitReached() {
        return false; // Encoders.ENABLED.get() && getPosition() <= 0;
    }

    /*** HOOK CLEARANCE ***/

    public boolean getLeftClear() {
        return !left.get();
    }

    public boolean getRightClear() {
        return !right.get();
    }

    public boolean getHookClear() {
        return getRightClear() && getLeftClear();
    }

    /*** STALL PROTECTION ***/

    private double getDutyCycle() {
        return climber.get();
    }

    private double getCurrentAmps() {
        return Math.abs(climber.getOutputCurrent());
    }

    public boolean isStalling() {
        boolean current = getCurrentAmps() > Stalling.CURRENT_THRESHOLD;
        boolean output = Math.abs(getDutyCycle()) > Stalling.DUTY_CYCLE_THRESHOLD;
        boolean velocity = Math.abs(getVelocity()) < Stalling.SCIBORGS_THRESHOLD;
        return Stalling.ENABLED.get() && stalling.calculate(output && current && velocity);
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
            // SmartDashboard.putNumber("Debug/Climber/Position", encoder.getPosition());

            SmartDashboard.putBoolean(
                    "Debug/Climber/Max Tilt", tilter.get().equals(Value.kReverse));
            SmartDashboard.putNumber("Debug/Climber/Climber Speed", climber.get());
        }
    }
}
