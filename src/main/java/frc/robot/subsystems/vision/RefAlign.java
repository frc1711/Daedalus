/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pixy2api.Pixy2CCC.Block;

/***
 * Shoutout to team 4546.
* @author: Lou DeZeeuw 
 */
public class RefAlign {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static ArrayList<Block> blocks = CameraConfig.getPixyCamera().getPixy().getCCC().getBlocks();
  private static final int blockSignature = 1; 
  public static Block largestBlock = null; 
  public static double yaw; 
  public static int threeChoice; 

  public static void run(int count) {
    SmartDashboard.putNumber("count", count);
    if (count > 0) {
      Block largestBlock = null;  
      for (Block block : blocks) {
        if(block.getSignature() == blockSignature) {
          if (largestBlock == null) 
            largestBlock = block; 
          else if (block.getWidth() > largestBlock.getWidth()) 
            largestBlock = block; 
        }
      }

      int ballX = largestBlock.getX();
      yaw = ((ballX - 157.5) * 0.1904761905);

      SmartDashboard.putNumber("Tape Angle", yaw);
      SmartDashboard.putNumber("Tape X", largestBlock.getX());
      SmartDashboard.putNumber("Tape Y", largestBlock.getY());
      SmartDashboard.putNumber("Tape Box Width", largestBlock.getWidth());
      SmartDashboard.putNumber("Tape Box Height", largestBlock.getHeight());
    }
  }
  

}
