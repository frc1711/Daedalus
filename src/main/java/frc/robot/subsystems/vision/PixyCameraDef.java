/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pixy2api.Pixy2;
import frc.robot.pixy2api.Pixy2CCC;
import frc.robot.pixy2api.links.Link;

/**
 * Add your docs here.
 */
public class PixyCameraDef extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static Pixy2 pixyCam; 
  public static int PixyResult = 0; 
  public static int initFailure = 0; 
  public PixyCameraDef(Link link) {
    pixyCam = Pixy2.createInstance(link); 
    pixyCam.init(); 
  }

  public void run() { 
    if (initFailure < 5) {
    //final int pixyStatus = pixyCam.init(PixyResult); 
   // int count = pixyCam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25); 
    int count = pixyCam.getCCC().getBlocks(false, Pixy2CCC.CCC_COLOR_CODES, 25); 
    //  int refCount = pixyCam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
     // int ballCount = pixyCam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25); 
      //int bumperBlueCount = pixyCam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 25); 
      SmartDashboard.putNumber("COUNT", count); 
      if (count >= 0) {
        BallTrack.run(count); 
        //BallFollow.run(); 

        SmartDashboard.putBoolean("PIXY RUNNING", true); 
       
        if (count == 0) 
          SmartDashboard.putBoolean("BALL VISIBLE", false); 
        else if (count > 0)
          SmartDashboard.putBoolean("BALL VISIBLE", true); 

       // ReflectiveAlign.run(count); 
       //RefAlign.run(refCount); 
       //LineUp.run(); 

      } else {
        if(pixyCam.init(PixyResult) != Pixy2.PIXY_RESULT_OK) {
          initFailure++; 
          System.out.println("PIXY NOT RUNNING");
          SmartDashboard.putBoolean("PIXY RUNNING", false); 
        } else {
          initFailure = 0; 
        }
      }
    } 
  }

  public Pixy2 getPixy() {
    return pixyCam; 
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
