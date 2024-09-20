/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.streams.numbers.filters.IFilter;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Conveyor.Direction;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
/*-
 * Intake balls into the robot
 *
 * Contains:
 *      - Double Solenoid to lift and lower intake
 *      - Motor to control rollers that intake the balls
 *
 * @author Elaine Ye
 * @author Ben Goldfisher
 * @author Allison Choi
 * @author Brandon Chao
 * @author Crystal Yang
 * @author Mandy Wang
 * @author Vincent Zheng
 * @author Naf Murtaza
 * @author Orion Xiang
 * @author Ethan Liu
 * @author Raymond Lian
 * @author Reya Miller
 */
public class Intake extends SubsystemBase {

    private final CANSparkMax motor;
    private final DoubleSolenoid solenoid;

    private final Conveyor conveyor;

    private final IFilter speedFilter;
    private double speed;

    private boolean ignoreConveyor;

    public Intake(Conveyor conveyor) {
        this.motor = new CANSparkMax(Ports.Intake.MOTOR, MotorType.kBrushless);
        Motors.INTAKE.configure(motor);

        this.solenoid =
                new DoubleSolenoid(
                        PneumaticsModuleType.CTREPCM,
                        Ports.Intake.SOLENOID_FORWARD,
                        Ports.Intake.SOLENOID_REVERSE);

        this.conveyor = conveyor;

        this.speedFilter = new LowPassFilter(Settings.Intake.SPEED_FILTERING);
        this.speed = 0.0;

        ignoreConveyor = false;
    }

    /*** Extend / Retract ***/
    public void extend() {
        solenoid.set(Value.kForward);
    }

    public void retract() {
        solenoid.set(Value.kReverse);
    }

    /*** Acquire / Deaqcuire ***/
    public void setMotor(double speed) {
        this.speed = speed;
    }

    public void stop() {
        this.speed = 0.0;
    }

    public void acquire() {
        this.speed = +Settings.Intake.ACQUIRE_SPEED.get();
    }

    public void deacquire() {
        if (DriverStation.isAutonomous()) {
            // this is here only to ensure we don't change behavior 
            // in autonomous mode. 
            this.speed = -Settings.Intake.ACQUIRE_SPEED.get();
        } else {
            this.speed = Settings.Intake.DEACQUIRE_SPEED.get();
        }
    }

    /*** Automated Intake Actions ***/
    public void setIgnoreConveyor(boolean disabled) { 
        ignoreConveyor = disabled;
    }

    private boolean getShouldStop() {
        if (ignoreConveyor) return false;
        return conveyor.getGandalfDirection() == Direction.STOPPED && conveyor.hasAnyBall();
    }

    private boolean getShouldSlow() {
        if (ignoreConveyor) return false;
        return conveyor.getGandalfDirection() != Direction.STOPPED && conveyor.hasAnyBall();
    }

    public boolean getShouldRetract() {
        if (ignoreConveyor) return false;
        return Settings.Intake.AUTO_RETRACT.get()
                && !DriverStation.isAutonomous()
                && conveyor.isFull();
    }

    /*** Debug Information ***/
    @Override
    public void periodic() {
        double motorSpeed = speedFilter.get(speed);
        if (0.0 <= motorSpeed && getShouldStop()) {
            motor.stopMotor();
        } else if (0.0 <= motorSpeed && getShouldSlow()) {
            motor.set(motorSpeed * 0.75);
        } else {
            motor.set(motorSpeed);
        }

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("Debug/Intake/Motor Speed", motor.get());
            SmartDashboard.putBoolean("Debug/Intake/Extended", solenoid.get() == Value.kForward);
        }
    }
}
