/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.links.Link;

/***
 * Shoutout to team 4546 
 * @author: Lou DeZeeuw 
 */

public class PixyCamera {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final Pixy2 pixyCam;
  private boolean lastState = false; 
  public final static int PixyResult = 0;

  public PixyCamera(Link link) {
    pixyCam = Pixy2.createInstance(link); 
    pixyCam.init();
  }

  public PixyCamera(Link link, int arg) {
    pixyCam = Pixy2.createInstance(link); 
    pixyCam.init(arg); 
  }
  
  public void run() {
    final int pixyStatus = pixyCam.init(PixyResult); 
    if (pixyStatus == 0) {
      
      final int refCount = pixyCam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG3, 2); 
      final int count = pixyCam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25); 
      BallTrack.run(count); 
      BallFollow.run(); 
//      ReflectiveAlign.run(refCount); 
    }
  }

  public void light(boolean state) {
    final int pixyStatus = pixyCam.init(PixyResult); 
    if (pixyStatus == 0) {

      if ((state == true) && (lastState == false)) {
        lastState = true; 
        pixyCam.setLamp((byte) 1, (byte) 0); 
      } else if (state == false && lastState == true) {
        lastState = false; 
        pixyCam.setLamp((byte) 0, (byte) 0); 
      }
      
    }

  }

  public Pixy2 getPixy() {
    return pixyCam; 
  }


}
