package org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RobotVoltageSensor {
    private VoltageSensor sensor;

    /**
     * Gets the voltage sensor from the hardware map
     *
     * @param hardwareMap The OpMode hardware map
     */
    public RobotVoltageSensor(HardwareMap hardwareMap) {
        for (com.qualcomm.robotcore.hardware.VoltageSensor _sensor : hardwareMap.voltageSensor) {
            sensor = _sensor;
        }
    }

    /**
     * Returns the voltage
     *
     * @return Returns the voltage in Volts
     */
    public double getVoltage() {
        return sensor.getVoltage();
    }
}
