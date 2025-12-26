package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Scanner;

public class ModeFileHandling {

    private static final String MODE_FILE_PATH = "/home/lvuser/isPit.txt";

    /**
     *
     * @return if the file for pit mode exists
     */
    public static boolean isPitMode() {
        File file = new File(MODE_FILE_PATH);
        return file.exists();
    }

    /**
     * deletes the pit mode file
     */
    public static void switchToComp(){
        File file = new File(MODE_FILE_PATH);
        file.delete();
    }

    /**
     * creates the pit mode file
     */
    public static void switchToPit(){
        File file = new File(MODE_FILE_PATH);
        try{
            file.createNewFile();
        } catch (IOException e){
            e.printStackTrace();
        }
    }
}
