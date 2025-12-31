package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.File;

public class ModeFileHandling {
    private static boolean wasFmsChecked = false;
    private static boolean fmsAttached;
    private static final String MODE_FILE_PATH = "/home/lvuser/isComp.txt";

    /**
     *
     * @return if file for comp mode exists
     */
    public static boolean isCompMode(){
        File file = new File(MODE_FILE_PATH);
        return file.exists();
    }
    /**
     * deletes the comp mode file
     */
    public static void switchToPitMode() {
        File file = new File(MODE_FILE_PATH);
        file.delete();

    }

    /**
     *
     * @return if robot should switch to pit mode
     */
    public static boolean shouldSwitchToPitMode() {

        if(!wasFmsChecked){
            fmsAttached = DriverStation.isFMSAttached();
            wasFmsChecked = true;
        }

        if(fmsAttached) return false;

        //TODO: set actual check for pit mode
        return true;
    }

}
