/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.util.Units;

import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.util.Interpolator;
import com.stuypulse.robot.util.NearestInterpolator;

/** Contains interpolation tables for shooting and alignment */
public interface ShotMap {
        
        double MIN_DISTANCE = 0;

        public interface Distances{
                double RING = Limelight.RING_DISTANCE.get();
                double POINT_A = Units.inchesToMeters(167);
                double POINT_B = Units.inchesToMeters(184);
                double POINT_C = Units.inchesToMeters(201);
                double LAUNCHPAD = Limelight.PAD_DISTANCE.get();
        }       

        // Converts a distance measurement to an RPM to shoot at
        Interpolator DISTANCE_TO_RPM =
                new NearestInterpolator(
                        NearestInterpolator.Bias.kCenter,
                        new Vector2D(Distances.RING, Settings.Shooter.RING_RPM.get()),
                        new Vector2D(Distances.POINT_A, 3175),
                        new Vector2D(Distances.POINT_B, 3325),
                        new Vector2D(Distances.POINT_C, 3500),
                        new Vector2D(Distances.LAUNCHPAD, Settings.Shooter.PAD_RPM.get()));

        public static double getRPM(double distance) {
            return DISTANCE_TO_RPM.interpolate(distance);
        }

        // Converts a distance measurement to an angle offset to align to
        Interpolator DISTANCE_TO_YAW =
                new NearestInterpolator(
                        NearestInterpolator.Bias.kCenter,
                        new Vector2D(Distances.RING, Limelight.RING_YAW.get()),
                        new Vector2D(Distances.POINT_A, 5.1),
                        new Vector2D(Distances.POINT_B, 5.70),
                        new Vector2D(Distances.POINT_C, 6.0),
                        new Vector2D(Distances.LAUNCHPAD, Limelight.PAD_YAW.get()));
        
        public static Angle getAngle(double distance) {
            return Angle.fromDegrees(DISTANCE_TO_YAW.interpolate(distance));
        }
                
}