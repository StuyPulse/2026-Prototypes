/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Intake { //TODO: !! Make suppliers and Smart numbers so that it can be edited through glass
        static double INTAKE = 1;
        static double STOW = 0;
        static double OUTTAKE = -1; //TODO: remove if we aren't doing outtake

        static double UP = 1; //TODO: update values
        static double DOWN = -1; //TODO: update values
        static double STOP = 0;
    }
}
