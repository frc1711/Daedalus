/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pixy2api.links.SPILink;

/***
 *Shoutout to team 4546
 * @author: Lou DeZeeuw 
 */

public class CameraConfig {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static PixyCameraDef pixyCam = null; 
  
  public static void setup() {
    pixyCam = new PixyCameraDef(new SPILink());
    System.out.println("PIXY CREATED");  
    SmartDashboard.putBoolean("PIXY CREATED", true);
  }
  
 /* public static void light (boolean state) {
    pixyCam.light(state); 
  } */
  public static void run() {
    pixyCam.run(); 
  }

  public static PixyCameraDef getPixyCamera() {
    return pixyCam; 
  }
}