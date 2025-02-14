package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * HuskyLensManager wraps the DFRobot/GoBilda HuskyLens sensor to provide a higher-level interface
 * for object detection and data retrieval over I2C.
 *
 * IMPORTANT:
 *   1. Ensure your sensor's I2C address is correct.
 *   2. The methods getData() and seesObject() should be adjusted based on the actual
 *      HuskyLens protocol commands you need to read or write.
 */
public class HuskyLensManager {
    // Reference to the underlying hardware driver (if included in your SDK).
    private HuskyLens huskyLens;
    
    // The raw I2C device interface.
    private I2cDeviceSynch i2cDeviceSynch;

    /**
     * Some HuskyLens devices default to 0x32 as the 7-bit address,
     * which is 0x64 in 8-bit form.
     * Check your lens documentation or test scanning addresses.
     */
    private static final I2cAddr HUSKYLENS_DEFAULT_I2C_ADDRESS = I2cAddr.create7bit(0x32);

    /**
     * Example read length or register constants. Most of the "GoBilda" and "DFRobot" HuskyLens
     * commands revolve around sending command frames like 0x55 0xAA and then reading the response.
     * You may need to change these if your device commands differ.
     */
    private static final int DEFAULT_READ_LEN = 16; // Example length to read back

    /**
     * Constructor that initializes the HuskyLens using the DFRobot/GoBilda sensor driver.
     *
     * @param hardwareMap The hardware map to access the I2C device.
     * @param deviceName  The configured name for the HuskyLens in the robot configuration.
     */
    public HuskyLensManager(HardwareMap hardwareMap, String deviceName) {
        // Retrieve the I2C device via its configured name in the Robot Configuration.
        i2cDeviceSynch = hardwareMap.i2cDeviceSynch.get(deviceName);

        // Optionally set the I2C address if needed (some drivers let you specify below).
        i2cDeviceSynch.setI2cAddress(HUSKYLENS_DEFAULT_I2C_ADDRESS);

        // Now instantiate the HuskyLens object. 
        // This is only valid if the SDK or library provides "com.qualcomm.hardware.dfrobot.HuskyLens".
        huskyLens = new HuskyLens(i2cDeviceSynch);

        // If your sensor requires any specific init or commands, do so here.
        // e.g. huskyLens.init() if that method exists, or set a mode.
        // Example (pseudo-code):
        // huskyLens.setAlgorithm(HuskyLens.Algorithm.FACE_RECOGNITION);
    }

    /**
     * Example method to check if at least one object is detected.
     * This is a stub that would actually need to parse returned frames from the HuskyLens.
     *
     * @return true if an object is detected, false otherwise.
     */
    public boolean seesObject() {
        // If the HuskyLens class provides a built-in method, you can call it directly:
        // return huskyLens.available() > 0; // (Hypothetical example)

        // Otherwise, you'd manually send a request frame, then parse the response frame.
        // For demonstration, here's a trivial read attempt:
        try {
            byte[] response = i2cDeviceSynch.read(0x00, DEFAULT_READ_LEN);
            // parse the response according to the HuskyLens protocol.
            // Hypothetically, say the first byte is a count:
            int objectCount = response[0] & 0xFF;
            return (objectCount > 0);
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Retrieves HuskyLens data for debugging. Typically you'd parse bounding boxes or IDs.
     *
     * @return A string containing sensor data for debugging.
     */
    public String getData() {
        try {
            // Example read of a chunk from register 0x00
            byte[] response = i2cDeviceSynch.read(0x00, DEFAULT_READ_LEN);

            // Convert to hex for debugging:
            StringBuilder sb = new StringBuilder();
            sb.append("Raw I2C Data: ");
            for (byte b : response) {
                sb.append(String.format("%02X ", b));
            }
            return sb.toString();
        } catch (Exception e) {
            return "Error reading sensor data.";
        }
    }
}
