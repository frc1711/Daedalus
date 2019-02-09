/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import io.github.pseudoresonance.pixy2api.links.SPILink;

/***
 *Shoutout to team 4546
 * @author: Lou DeZeeuw 
 */

public class CameraConfig {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static PixyCamera pixyCam = null; 
  
  public static void setup() {
    pixyCam = new PixyCamera(new SPILink()); 
  }
  
  public static void light (boolean state) {
    pixyCam.light(state); 
  }
  public static void run() {
    pixyCam.run(); 
  }

  public static PixyCamera getPixyCamera() {
    return pixyCam; 
  }
}
