/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Moves the robot around
 *
 * Contains: 
 *      - 3 motors on left side 
 *      - 3 motors on right side 
 *      - Encoders for both sides 
 *      - Gear shifting solonoid 
 *      - NavX / Gyroscope
 *
 * @author Myles Pasetsky
 * @author Sam Belliveau
 */
public class Drivetrain extends SubsystemBase {

    public Drivetrain() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
