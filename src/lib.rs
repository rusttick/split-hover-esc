//! Split Hover ESC - Remote UART Bus protocol implementation for RP2040
#![no_std]

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::InterruptHandler as UartInterruptHandler;

bind_interrupts!(pub struct Irqs {
    UART0_IRQ => UartInterruptHandler<UART0>;
});

/// Remote UART Bus protocol constants
pub mod protocol {
    /// Start byte for server->hoverboard messages
    pub const START_BYTE: u8 = b'/';

    /// Start frame for hoverboard->server responses (0xABCD little-endian)
    pub const RESPONSE_START: u16 = 0xABCD;

    /// Message type 0: Basic speed control
    pub const MSG_TYPE_SPEED: u8 = 0;

    /// Message type 1: Master mode with steering
    pub const MSG_TYPE_MASTER: u8 = 1;

    /// Message type 2: Configuration
    pub const MSG_TYPE_CONFIG: u8 = 2;

    /// Default slave ID
    pub const DEFAULT_SLAVE_ID: u8 = 1;

    /// Size of Type 0 message (basic speed control)
    pub const MSG_TYPE0_SIZE: usize = 8;

    /// Size of response message
    pub const RESPONSE_SIZE: usize = 15;

    /// CRC-16/CCITT calculation
    /// Polynomial: 0x1021, Initial: 0, No final XOR
    pub fn calc_crc16(data: &[u8]) -> u16 {
        let mut crc: u16 = 0;
        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }

    /// Build a Type 0 speed control message
    /// Returns the complete 8-byte message with CRC
    pub fn build_speed_msg(slave_id: u8, speed: i16, state: u8) -> [u8; MSG_TYPE0_SIZE] {
        let mut msg = [0u8; MSG_TYPE0_SIZE];
        msg[0] = START_BYTE;
        msg[1] = MSG_TYPE_SPEED;
        msg[2] = slave_id;
        msg[3] = (speed & 0xFF) as u8;         // speed low byte
        msg[4] = ((speed >> 8) & 0xFF) as u8;  // speed high byte
        msg[5] = state;

        // CRC over first 6 bytes, stored little-endian (firmware expects low byte first)
        let crc = calc_crc16(&msg[0..6]);
        msg[6] = (crc & 0xFF) as u8; // CRC low byte
        msg[7] = (crc >> 8) as u8;   // CRC high byte

        msg
    }

    /// Parsed response from hoverboard
    #[derive(Debug, Clone, Copy, defmt::Format)]
    pub struct Response {
        pub slave_id: u8,
        /// Speed in 0.1 RPM units (divide by 10 for RPM)
        pub speed: i16,
        /// Battery voltage in millivolts
        pub voltage_mv: u16,
        /// Current in 0.01A units (divide by 100 for amps)
        pub current_ca: i16,
        /// Odometer in hall sensor steps
        pub odometer: i32,
    }

    /// Parse a response message
    /// Returns None if invalid start frame or CRC mismatch
    pub fn parse_response(data: &[u8; RESPONSE_SIZE]) -> Option<Response> {
        // Check start frame (0xABCD little-endian: 0xCD, 0xAB)
        if data[0] != 0xCD || data[1] != 0xAB {
            return None;
        }

        // Verify CRC (over first 13 bytes, CRC in bytes 13-14 little-endian)
        let crc_calc = calc_crc16(&data[0..13]);
        let crc_recv = (data[13] as u16) | ((data[14] as u16) << 8);
        if crc_calc != crc_recv {
            return None;
        }

        Some(Response {
            slave_id: data[2],
            speed: (data[3] as i16) | ((data[4] as i16) << 8),
            voltage_mv: (data[5] as u16) | ((data[6] as u16) << 8),
            current_ca: (data[7] as i16) | ((data[8] as i16) << 8),
            odometer: (data[9] as i32)
                | ((data[10] as i32) << 8)
                | ((data[11] as i32) << 16)
                | ((data[12] as i32) << 24),
        })
    }
}
