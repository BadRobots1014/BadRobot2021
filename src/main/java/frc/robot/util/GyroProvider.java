/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants.DriveConstants;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * Add your docs here.
 */
public class GyroProvider {
    
    private Gyro m_simGyro;
    private AHRS m_realGyro;
    private boolean m_isReal = false;

    public GyroProvider(final boolean isReal) {
        m_isReal = isReal;
        if (isReal) {
            m_realGyro = new AHRS(Port.kMXP);
            m_realGyro.reset();
        } else {
            m_simGyro = new ADXRS450_Gyro(Port.kMXP);
            m_simGyro.calibrate();
        }
    }

    /**
     * 
     * @return Gets the heading [-180,180]
     */
    public double getHeading() {
        if (m_isReal) {
            return Math.IEEEremainder(m_realGyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        } else {
            return Math.IEEEremainder(m_simGyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        }
    }

    public double getRawAngle() {        
        if (m_isReal) {
            return m_realGyro.getAngle();
        } else {
            return m_simGyro.getAngle();
        }
    }

    public void reset() {
        if (m_isReal) {
            m_realGyro.reset();
        } else {
            m_simGyro.reset();
        }
    }

    public double getTurnRate() {
        if (m_isReal) {
            return m_realGyro.getRate();
        } else {
            return m_simGyro.getRate();
        }
    }
}