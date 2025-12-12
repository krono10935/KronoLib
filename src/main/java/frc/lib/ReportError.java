package frc.lib;

import java.util.ArrayList;

public class ReportError{

    private final ArrayList<ErrorMessage> errorLog = new ArrayList<>();
    private static ReportError instance;

    private ReportError(){
        // private constructor to prevent instantiation
    }

    /**
     * @return the singleton instance of the ReportError class
     */
    public static ReportError getInstance(){
        if(instance == null){
            instance = new ReportError();
        }
        return instance;
    }

    /**
     * Adds an ErrorMessage record to the list of Errors to check
     * @param error
     */
    public void addError(ErrorMessage error){
        errorLog.add(error);
    }
}
