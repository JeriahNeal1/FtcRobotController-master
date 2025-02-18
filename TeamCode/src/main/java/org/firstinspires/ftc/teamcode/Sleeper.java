package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

/**
 * Utility class for sleeping.
 *
 * This sleep method is safe to use on an FTC Robot running Android.
 */
public class Sleeper {
    /**
     * Sleeps for the specified amount of time in milliseconds.
     *
     * @param millis The number of milliseconds to sleep.
     */
    public static void sleep(long millis) {
        SystemClock.sleep(millis);
    }
}