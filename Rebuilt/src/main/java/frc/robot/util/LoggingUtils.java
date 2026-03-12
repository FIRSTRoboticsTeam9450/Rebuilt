// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.littletonrobotics.junction.Logger;

/** Misc utility class. */
public class LoggingUtils {
    /** Don't put a '/' at the end of the path */
    public static void logSecondsAsTime(String path, int seconds) {
        Logger.recordOutput(path + "/TotalSeconds", seconds);
        
        int hours = seconds / (60 * 60); seconds -= hours * (60 * 60);
        int minutes = seconds / 60; seconds -= minutes * 60;
        Logger.recordOutput(path + "/Hours", hours);
        Logger.recordOutput(path + "/Minutes", minutes);
        Logger.recordOutput(path + "/Seconds", seconds);
    }
}
