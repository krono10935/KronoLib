package frc.lib;

import java.util.ArrayList;

public class ReportError{

    private final ArrayList<ErrorMessage> errorLog = new ArrayList<>();
    private static ReportError instance;

    private ReportError(){
        // private constructor to prevent instantiation
    }

    public static ReportError getInstance(){
        if(instance == null){
            instance = new ReportError();
        }
        return instance;
    }

    public void addError(ErrorMessage error){
        errorLog.add(error);
    }

    public void SendErrors(){
        errorLog.forEach(ErrorMessage::update);
    }
}
