package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Scanner;

public class ModeFileHandling {

    private static final String MODE_FILE_PATH = "/home/lvuser/isComp.txt";

    /**
     *
     * @return if the file for comp mode exists
     */
    public static boolean isCompMode() {
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


}
