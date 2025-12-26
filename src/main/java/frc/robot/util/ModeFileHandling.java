package frc.robot.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Scanner;

public class ModeFileHandling {

    private static final String MODE_FILE_PATH = "/home/lvuser/robot_mode.txt";

    public enum RobotMode{
        PIT(true),COMP(true), UNKNOWNMODE(false), NOMODEFILE(false);

        final boolean valid;

        RobotMode(boolean valid){
            this.valid = valid;
        }
    }


    /**
     * Reads the first line of the robot mode file.
     * @return The first line of the mode file as a {@link String}, or {@code "UnknownMode"} if
     *         the file is missing or empty.
     */
    public static RobotMode getModeFromFile() {
        String mode = "";
        try (Scanner scanner = new Scanner(new File(MODE_FILE_PATH))) {
            if (scanner.hasNextLine()) {
                mode = scanner.nextLine();
            }
        } catch (FileNotFoundException e) {
            // File doesn't exist, return default
            return RobotMode.NOMODEFILE;
        }
        for (RobotMode robotmode : RobotMode.values()) {

            if(mode.equals(robotmode.name()) && robotmode.valid) return robotmode;
        }
        return RobotMode.UNKNOWNMODE;
    }


    private static void createFileWithSingleLine(String filePath, String line) {
        try (PrintWriter writer = new PrintWriter(filePath)) {
            writer.println(line); // writes a single line
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    /**
     * Handles the mode switch by creating a new mode file with the correct mode.
     * @param mode the mode in which the robot is in
     */
    public static void handleModeSwitch(RobotMode mode){
        switch(mode){
            case PIT:
                createFileWithSingleLine(MODE_FILE_PATH, RobotMode.COMP.name());
                throw new RuntimeException("Robot is in pit mode. switching to comp mode.");

            case COMP:
                createFileWithSingleLine(MODE_FILE_PATH, RobotMode.PIT.name());
                throw new RuntimeException("Robot is in comp mode. switching to pit mode.");

            case UNKNOWNMODE:
                createFileWithSingleLine(MODE_FILE_PATH, RobotMode.PIT.name());
                throw new RuntimeException("Unknown mode file. created mode file with default value: pit");
            case NOMODEFILE:
                createFileWithSingleLine(MODE_FILE_PATH, RobotMode.PIT.name());
                throw new RuntimeException("No mode file found. created mode file with default value: pit");
            default: break;
        }
    }
}
