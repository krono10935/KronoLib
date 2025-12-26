package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.File;

public class ModeFileHandling {
    private static boolean wasFmsChecked = false;
    private static boolean fmsAttached;
    private static final String MODE_FILE_PATH = "/home/lvuser/isComp.txt";


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
        //TODO: set actual check
        return !fmsAttached && true;
    }

}
