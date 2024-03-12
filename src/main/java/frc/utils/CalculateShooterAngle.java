// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

/** Add your docs here. */
public class CalculateShooterAngle {
    public double calculateShooterAngle(double robotDistance) 
    {
        double maxHight = 2;
        double desiredAngle;

        desiredAngle = Math.atan(maxHight/robotDistance);

        return desiredAngle;
    }
}