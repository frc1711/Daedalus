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
/**
 * Add your docs here.
 */

public class ReflectiveAlign {
    private static ArrayList<Block> blocks = CameraConfig.getPixyCamera().getPixy().getCCC().getBlocks();
    private static final int blockSignature = 1; 
    public static Block largestBlock = null; 
    public static Block smallestBlock = null; 
    public static double yaw; 
    public static double tapeXLarge; 
    public static double tapeXSmall; 

    public static void run(int count) {
        SmartDashboard.putNumber("Count", count); 

        if (count > 0 ) {
            Block largestBlock = null; 
            Block smallestBlock = null; 
            for (Block block : blocks) {
                if(block.getSignature() == blockSignature) {
 
                    //i think this needs to be changed but who knows
                    if (largestBlock == null) {
                        SmartDashboard.putBoolean("BLOCK SIG SET TWO", true); 
                        largestBlock = block; 
                    }
                    else if (block.getWidth() > largestBlock.getWidth()) {
                        largestBlock = block;
                        tapeXLarge = largestBlock.getX();
                    }    
                    else if (largestBlock != null && block.getWidth() < largestBlock.getWidth()){
                        smallestBlock = block; 
                        tapeXSmall = smallestBlock.getX(); 
                    } 
            } 
            }
        }

            
            double averageX = (tapeXLarge + tapeXSmall) / 2; 
            double yaw = ((averageX - 157.5) * 0.1904761905);
            SmartDashboard.putNumber("YAW", yaw); 
            if (largestBlock != null) {
                SmartDashboard.putNumber("Large X, Tape", largestBlock.getX()); 
                SmartDashboard.putNumber("Small X, Tape", smallestBlock.getX()); 
                SmartDashboard.putNumber("Average X, Tape", averageX);
                SmartDashboard.putNumber("Tape Angle?", yaw); 
                SmartDashboard.putNumber("Large Tape Width", largestBlock.getWidth()); 
                SmartDashboard.putNumber("Small Tape Width", smallestBlock.getWidth());
                SmartDashboard.putNumber("Large Tape Height", largestBlock.getHeight());
                SmartDashboard.putNumber("Small Tape Height", smallestBlock.getHeight()); 
        } 
    }

} 
