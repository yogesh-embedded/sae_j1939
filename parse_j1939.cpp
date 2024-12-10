#include "Arduino.h"
#include "ARD1939.h"


void parseJ1939Message(PGN pgn, const uint8_t* pMsg, int nMsgLen, J1939Data& data) {
    switch (pgn) {
        case PGN_ET1: // Engine Temperature 1
            if (nMsgLen >= 8) {
                data.et1.coolant_temp = pMsg[0] - 40;
                data.et1.fuel_temp = pMsg[1] - 40;
                data.et1.oil_temp = (pMsg[2] | (pMsg[3] << 8)) * 0.03125 - 273;
                data.et1.turbo_oil_temp = (pMsg[4] | (pMsg[5] << 8)) * 0.03125 - 273;
                data.et1.intercooler_temp = pMsg[6] - 40;
                data.et1.intercooler_thermostat_opening = pMsg[7];
            }
            break;

        case PGN_AMB: // Ambient Conditions
            if (nMsgLen >= 8) {
                data.amb.barometric_pressure = pMsg[0];
                data.amb.cab_interior_temp = pMsg[1] | (pMsg[2] << 8);
                data.amb.ambient_air_temp = pMsg[3] | (pMsg[4] << 8);
                data.amb.air_inlet_temp = pMsg[5] - 40;
                data.amb.road_surface_temp = pMsg[6] | (pMsg[7] << 8);
            }
            break;

        case PGN_LFC: // Fuel Consumption
            if (nMsgLen >= 8) {
                data.lfc.engine_trip_fuel = pMsg[0] | (pMsg[1] << 8) | (pMsg[2] << 16) | (pMsg[3] << 24);
                data.lfc.engine_total_fuel_used = pMsg[4] | (pMsg[5] << 8) | (pMsg[6] << 16) | (pMsg[7] << 24);
            }
            break;

        case PGN_EEC1: // Electronic Engine Controller 1
            if (nMsgLen >= 8) {
                data.eec1.engine_torque_mode = pMsg[0] & 0x0F;
                data.eec1.driver_percent_torque = pMsg[1];
                data.eec1.actual_percent_torque = pMsg[2];
                data.eec1.engine_speed = (pMsg[3] | (pMsg[4] << 8)) * 0.125;
                data.eec1.source_address = pMsg[5];
                data.eec1.starter_mode = pMsg[6] & 0x0F;
                data.eec1.engine_demand_percent_torque = pMsg[7];
            }
            break;

        case PGN_EEC2: // Electronic Engine Controller 2
            if (nMsgLen >= 8) {
                // Extract Accelerator Pedal Switch (1st byte)
                data.eec2.accel_pedal_switch.low_idle_switch = (pMsg[0] >> 0) & 0x03; // Bits 1-2
                data.eec2.accel_pedal_switch.kickdown_switch = (pMsg[0] >> 2) & 0x03; // Bits 3-4
                data.eec2.accel_pedal_switch.speed_limit_status = (pMsg[0] >> 4) & 0x03; // Bits 5-6
                data.eec2.accel_pedal_switch.low_idle_switch_2 = (pMsg[0] >> 6) & 0x03; // Bits 7-8

                // Extract Accelerator Pedal Position 1 (SPN 91)
                data.eec2.accel_pedal_pos1 = pMsg[1];

                // Extract Engine Percent Load (SPN 92)
                data.eec2.engine_percent_load = pMsg[2];

                // Extract Remote Accelerator Pedal Position (SPN 974)
                data.eec2.remote_accel_pedal_pos = pMsg[3];

                // Extract Accelerator Pedal Position 2 (SPN 29)
                data.eec2.accel_pedal_pos2 = pMsg[4];

                // Extract Vehicle Acceleration Rate Limit Status (SPN 2979)
                data.eec2.accel_status.acceleration_rate_limit = (pMsg[5] >> 0) & 0x03; // Bits 1-2

                // Extract Actual Maximum Available - Percent Torque (SPN 3357)
                data.eec2.max_available_torque = pMsg[6];
            }
            break;

        case PGN_IC1: // Inlet/Exhaust Conditions 1
            if (nMsgLen >= 8) {
                // Extract SPNs from the payload
                data.ic1.particulate_trap_inlet_pressure = pMsg[0]; // SPN 81
                data.ic1.intake_manifold_pressure = pMsg[1];       // SPN 102
                data.ic1.intake_manifold_temperature = pMsg[2];   // SPN 105
                data.ic1.air_inlet_pressure = pMsg[3];            // SPN 106
                data.ic1.air_filter_differential_pressure = pMsg[4]; // SPN 107
                
                // Extract 16-bit Exhaust Gas Temperature (SPN 173)
                data.ic1.exhaust_gas_temperature = pMsg[5] | (pMsg[6] << 8);

                // Extract Coolant Filter Differential Pressure (SPN 112)
                data.ic1.coolant_filter_differential_pressure = pMsg[7];
            }
            break;

        case PGN_TCO1: // Tachograph
            if (nMsgLen >= 8) {
                uint8_t byte0 = pMsg[0];
                uint8_t byte1 = pMsg[1];
                uint8_t byte2 = pMsg[2];
                uint8_t byte3 = pMsg[3];

                // Map bytes directly into bit fields
                data.tco1.driver1_working_state = byte0 & 0x07;          // SPN 1612
                data.tco1.driver2_working_state = (byte0 >> 3) & 0x07;  // SPN 1613
                data.tco1.vehicle_motion = (byte0 >> 6) & 0x03;         // SPN 1611

                data.tco1.driver1_time_related_states = byte1 & 0x0F;   // SPN 1617
                data.tco1.driver_card_driver1 = (byte1 >> 4) & 0x03;    // SPN 1615
                data.tco1.vehicle_overspeed = (byte1 >> 6) & 0x03;      // SPN 1614

                data.tco1.driver2_time_related_states = byte2 & 0x0F;   // SPN 1618
                data.tco1.driver_card_driver2 = (byte2 >> 4) & 0x03;    // SPN 1616

                data.tco1.system_event = byte3 & 0x03;                  // SPN 1622
                data.tco1.handling_information = (byte3 >> 2) & 0x03;  // SPN 1621
                data.tco1.tachograph_performance = (byte3 >> 4) & 0x03; // SPN 1620
                data.tco1.direction_indicator = (byte3 >> 6) & 0x03;    // SPN 1619

                // Extract 16-bit values
                data.tco1.output_shaft_speed = pMsg[4] | (pMsg[5] << 8); // SPN 1623
                data.tco1.vehicle_speed = pMsg[6] | (pMsg[7] << 8);      // SPN 1624
            }
            break;

        case PGN_65265:
            if (nMsgLen >= 8) {
                // SPN 84: Wheel-Based Vehicle Speed (Bytes 2-3, 16 bits, resolution 1/256 km/h)
                data.ccvs.wheel_based_vehicle_speed = ((pMsg[1] | (pMsg[2] << 8)) * 0.00390625); // Convert to km/h
            }
            break;

        case PGN_65248:
            if (nMsgLen >= 8) {
                // SPN 244: Trip Distance (Bytes 1-4, 32 bits)
                data.vehicle_distance_status.trip_distance = 
                    (uint32_t)pMsg[0]       | 
                    ((uint32_t)pMsg[1] << 8)  | 
                    ((uint32_t)pMsg[2] << 16) | 
                    ((uint32_t)pMsg[3] << 24);

                // SPN 245: Total Vehicle Distance (Bytes 5-8, 32 bits)
                data.vehicle_distance_status.total_vehicle_distance = 
                    (uint32_t)pMsg[4]       | 
                    ((uint32_t)pMsg[5] << 8)  | 
                    ((uint32_t)pMsg[6] << 16) | 
                    ((uint32_t)pMsg[7] << 24);
            }
            break;

        case PGN_65253:
            if (nMsgLen >= 8) {
                // SPN 247: Total Engine Hours (Bytes 1-4, 32 bits)
                data.engine_hours_revolutions.total_engine_hours = 
                    (uint32_t)pMsg[0]       | 
                    ((uint32_t)pMsg[1] << 8)  | 
                    ((uint32_t)pMsg[2] << 16) | 
                    ((uint32_t)pMsg[3] << 24);

                // SPN 249: Total Engine Revolutions (Bytes 5-8, 32 bits)
                data.engine_hours_revolutions.total_engine_revolutions = 
                    (uint32_t)pMsg[4]       | 
                    ((uint32_t)pMsg[5] << 8)  | 
                    ((uint32_t)pMsg[6] << 16) | 
                    ((uint32_t)pMsg[7] << 24);
            }
            break;

        case PGN_65276:
            if (nMsgLen >= 8) {

                // SPN 96: Fuel Level (Byte 2, 8 bits)
                data.dash_display.fuel_level = pMsg[1];
            }
            break;

        case PGN_65271:
            if (nMsgLen >= 8) {
                // SPN 158: Battery Potential (Voltage), Switched (Bytes 7-8, 16 bits, resolution 0.05 V)
                data.vehicle_electrical_power.battery_potential_voltage_switched =
                    (uint16_t)((pMsg[6] | (pMsg[7] << 8)) * 0.05);
            }
            break;

        case PGN_65263:
            if (nMsgLen >= 8) {
                // SPN 100: Engine Oil Pressure (Byte 4, 8 bits, resolution 1 kPa)
                data.engine_fluid_level_pressure.engine_oil_pressure = pMsg[3];  // Byte 4 holds Engine Oil Pressure
            } else {
                printf("Invalid message length for PGN 65263.\n");
            }
            break;

        case PGN_65266:
            if (nMsgLen >= 8) {
                // SPN 183: Fuel Rate (Bytes 1-2, 16 bits, resolution 0.1 L/h)
                data.fuel_economy_status.fuel_rate = (pMsg[0] | (pMsg[1] << 8)); // Combine Bytes 1 and 2
            } else {
                printf("Invalid message length for PGN 65266.\n");
            }
            break;

        case PGN_65244:
            if (nMsgLen >= 8) {
                // SPN 235: Total Idle Hours (Bytes 1-4, 32 bits)
                data.idle_operation_status.total_idle_hours = (pMsg[0] | (pMsg[1] << 8) | (pMsg[2] << 16) | (pMsg[3] << 24)); // Combine Bytes 1-4
            } else {
                printf("Invalid message length for PGN 65244.\n");
            }
            break;


        default:
            // Serial.println("PGN not recognized or implemented.");
            break;
    }
}



// Function to clear screen using VT100 escape codes
void clearScreen() {
    Serial.write(27); // ESC
    Serial.print("[2J"); // Clear screen
    Serial.write(27); // ESC
    Serial.print("[H"); // Move cursor to home
}

// Function to print dynamic data
void printDynamicData(const J1939Data& data) {
    clearScreen();
    Serial.println("Dynamic Data:");


    // Engine RPM
    Serial.print("Engine Coolant Temperature: ");
    Serial.println(data.et1.coolant_temp);

    // Engine RPM
    Serial.print("Engine RPM: ");
    Serial.println(data.eec1.engine_speed, 2);

    // Accelerator Pedal Position
    Serial.print("Accelerator Pedal Position: ");
    Serial.println(data.eec2.accel_pedal_pos1);

    // Intake Manifold Pressure
    Serial.print("Intake Manifold Pressure: ");
    Serial.println(data.ic1.intake_manifold_pressure);

    // Vehicle Speed
    Serial.print("Vehicle Speed: ");
    Serial.println(data.tco1.vehicle_speed);

    // Add a small delay to prevent excessive screen updates
    // delay(100);
}
