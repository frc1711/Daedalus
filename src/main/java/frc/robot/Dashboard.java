/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



//delete file later


/**
 * Add your docs here.
 */
public class Dashboard {

    private static Dashboard instance; 

    private boolean shouldDisplayField (String key) {
        return true; 
    }

    public boolean showDiagnostics; 

    public void init(boolean showDiagnostics) {
        this.showDiagnostics = showDiagnostics; 
    }

    public void putString (boolean isCompetition, String key, String value) {
        if (shouldDisplayField(key)) 
            SmartDashboard.putString(key, value); 
    }

    public void putNumber (boolean isCompetition, String key, double value) {
        if (shouldDisplayField(key)) 
            SmartDashboard.putNumber(key, value); 
    }

    public void putBoolean(boolean isCompetition, String key, boolean value) {
        if (shouldDisplayField(key)) 
            SmartDashboard.putBoolean(key, value); 
    }
    public void putData (boolean isCompetition, String key, Sendable value) {
        if (shouldDisplayField(key)) 
            SmartDashboard.putData(key, value);
        
    }
    
    public static Dashboard getInstance() { 
        if (instance == null) 
            instance = new Dashboard(); 
        return instance; 
    }
    
}
