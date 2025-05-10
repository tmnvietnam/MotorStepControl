#include <Arduino.h>
#include <SPI.h>
#include "df_can.h"

#define DEBUG

#define BAUD_RATE 115200
#define ReadWriteOneByte SPI.transfer
#define Read() SPI.transfer(0x00)

enum class status_code : uint8_t
{
  UNKNOWN_CMD = '0',
  ERROR_CMD,
  PENDING,
  CANCEL,
  ENDED,
  OVER_UPPER_LIMIT,
  OVER_LOWER_LIMIT,
  CAN_ERROR,
  REACHED_SENSOR,
  UNREACHED_SENSOR
};

void mode0(uint16_t can_id, uint8_t ena, uint8_t dir, uint32_t pulse_target, uint16_t pps);
void moveWithStep(uint32_t total_steps, uint8_t dir);
void sendCurrentStatus(status_code status_code, long position);

const uint16_t STEP_SIZE = 500;      // Steps per movement chunk
const uint16_t PULSE_PER_SEC = 1500; // Pulses per second

const int SPI_CS_PIN = 10;
const int LS_PIN = 8;
const int US_PIN = 8;

MCPCAN CAN(SPI_CS_PIN);

long current_position = 0;
bool stop_requested = false;
bool can_error = false;

void setup()
{
  Serial.begin(BAUD_RATE);
  while (!Serial)
    ;

  pinMode(LS_PIN, INPUT);
  pinMode(US_PIN, INPUT);

  int count = 50;
  do
  {
    CAN.init();
    if (CAN_OK == CAN.begin(CAN_1000KBPS))
    {
      mode0(0x001, 0, 0, 0, 0);

      break;
    }

    delay(100);
  } while (count--);
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("H"))
    {
      while (digitalRead(LS_PIN) == LOW)
      {
        mode0(0x001, 1, 1, STEP_SIZE, PULSE_PER_SEC);
        if (can_error)
        {
          sendCurrentStatus(status_code::CAN_ERROR, current_position);
          return;
        }
        current_position -= STEP_SIZE;
        sendCurrentStatus(status_code::PENDING, current_position);

        unsigned long start_time = millis();
        while (millis() - start_time < 100)
        {
          if (Serial.available())
          {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();
            if (cmd == "S")
            {
              stop_requested = true;
              mode0(0x001, 0, 0, 0, 0);
              sendCurrentStatus(status_code::CANCEL, current_position);
              return;
            }
          }
        }
      }
      mode0(0x001, 0, 0, 0, 0);
      current_position = 0;
      sendCurrentStatus(status_code::ENDED, current_position);
    }
    else if (command.startsWith("S"))
    {
      stop_requested = true;
      mode0(0x001, 0, 0, 0, 0);
      sendCurrentStatus(status_code::CANCEL, current_position);
    }
    else if (command.startsWith("U"))
    {
      String number_part = command.substring(1);
      if (number_part.length() > 0)
      {
        uint32_t num_step = number_part.toInt();
        stop_requested = false;
        moveWithStep(num_step, 0);
      }
      else
      {
        sendCurrentStatus(status_code::ERROR_CMD, current_position);
      }
    }
    else if (command.startsWith("D"))
    {
      String number_part = command.substring(1);
      if (number_part.length() > 0)
      {
        uint32_t num_step = command.substring(1).toInt();
        stop_requested = false;
        moveWithStep(num_step, 1);
      }
      else
      {
        sendCurrentStatus(status_code::ERROR_CMD, current_position);
      }
    }
    else if (command.startsWith("M"))
    {
      String number_part = command.substring(1);
      if (number_part.length() > 0)
      {
        uint32_t pwmTarget = command.substring(1).toInt();
        stop_requested = false;
        uint32_t diff = (pwmTarget > current_position) ? (pwmTarget - current_position) : (current_position - pwmTarget);
        uint8_t dir = (pwmTarget > current_position) ? 0 : 1;
        moveWithStep(diff, dir);
      }
      else
      {
        sendCurrentStatus(status_code::ERROR_CMD, current_position);
      }
    }
    else if (command.startsWith("P"))
    {
      sendCurrentStatus(status_code::ENDED, current_position);
    }
    else if (command.startsWith("L"))
    {
      if (command.length() == 2)
      {
        if (command[1] == 'L')
        {
          sendCurrentStatus(digitalRead(LS_PIN) ? status_code::REACHED_SENSOR : status_code::UNREACHED_SENSOR, current_position);
        }
        if (command[1] == 'U')
        {
          sendCurrentStatus(digitalRead(US_PIN) ? status_code::REACHED_SENSOR : status_code::UNREACHED_SENSOR, current_position);
        }
      }
      else
      {
        sendCurrentStatus(status_code::UNKNOWN_CMD, current_position);
      }
    }
    else
    {
      sendCurrentStatus(status_code::UNKNOWN_CMD, current_position);
    }
  }
}

/**
 * @brief Sends the current status code and position over the serial port.
 *
 * This function creates a 12-byte packet consisting of:
 * - 1 byte: ASCII character representing the status code (0–9)
 * - 11 bytes: formatted position as a signed 10-digit number with leading zeros
 *
 * The entire packet is then sent using `Serial.write()`.
 *
 * @param status_code The status code to send (0–9).
 * @param position The position value to send, formatted as a signed 10-digit string.
 */
void sendCurrentStatus(status_code status_code, long position)
{
  byte packet[12];

  // Status code as ASCII digit
  packet[0] = (byte)(status_code);

  // Temporary buffer for position string
  char posStr[12];
  snprintf(posStr, sizeof(posStr), "%+011ld", position);

  memcpy(&packet[1], posStr, 11);

  Serial.write(packet, sizeof(packet));

#ifdef DEBUG
  Serial.println();
#endif
}

/**
 * @brief Moves motor with specified steps and direction
 *
 * @param total_steps Total steps to move (0 for continuous)
 * @param dir Direction (0 = forward/up, 1 = backward/down)
 *
 * Features:
 * - Limit switch protection (both upper/lower)
 * - Emergency stop handling ('S' command)
 * - Position tracking with status updates
 * - Configurable step size and pulse rate
 * - Non-blocking delay for serial monitoring
 */
void moveWithStep(uint32_t total_steps, uint8_t dir)
{
  // Configuration

  uint32_t steps_remaining = total_steps;

  // Safety checks

  // Movement loop
  while (steps_remaining > 0)
  {
    if (stop_requested)
    {
      sendCurrentStatus(status_code::CANCEL, current_position);
      return;
    };

    if (dir == 0 && digitalRead(US_PIN) == HIGH)
    {
      sendCurrentStatus(status_code::OVER_UPPER_LIMIT, current_position);
      return;
    }

    if (dir == 1 && digitalRead(LS_PIN) == HIGH)
    {
      current_position = 0;
      sendCurrentStatus(status_code::OVER_LOWER_LIMIT, current_position);
      return;
    }

    uint16_t step = (steps_remaining >= STEP_SIZE) ? STEP_SIZE : steps_remaining;

    // Execute movement
    mode0(0x001, 1, dir, step, PULSE_PER_SEC);
    if (can_error)
    {
      sendCurrentStatus(status_code::CAN_ERROR, current_position);
      return;
    }

    // Non-blocking monitoring (100ms)
    unsigned long start_time = millis();
    while (millis() - start_time < 100)
    {
      if (Serial.available())
      {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "S")
        {
          stop_requested = true;
          mode0(0x001, 0, 0, 0, 0); // Emergency stop
          sendCurrentStatus(status_code::CANCEL, current_position);
          return;
        }
      }
    }

    // Update position
    if (dir == 0)
      current_position += step;
    else
      current_position -= step;
    sendCurrentStatus(status_code::PENDING, current_position);
    steps_remaining -= step;
  }
  sendCurrentStatus(status_code::ENDED, current_position);
  mode0(0x001, 0, 0, 0, 0);
}

void mcpSetRegister(const INT8U RegAddr, const INT8U value)
{
  digitalWrite(SPI_CS_PIN, LOW);
  ReadWriteOneByte(MCP_WRITE);
  ReadWriteOneByte(RegAddr);
  ReadWriteOneByte(value);
  digitalWrite(SPI_CS_PIN, HIGH);
}

INT8U mcpReadRegister(const INT8U RegAddr)
{
  INT8U ret;

  digitalWrite(SPI_CS_PIN, LOW);
  ReadWriteOneByte(MCP_READ);
  ReadWriteOneByte(RegAddr);
  ret = Read();
  digitalWrite(SPI_CS_PIN, HIGH);

  return ret;
}
void mcpReset()
{
  digitalWrite(SPI_CS_PIN, LOW);
  ReadWriteOneByte(MCP_RESET);
  digitalWrite(SPI_CS_PIN, HIGH);
  delay(100);
}

void mode0(uint16_t can_id, uint8_t ena, uint8_t dir, uint32_t pulse_target, uint16_t pps)
{
  uint8_t data[8];
  data[0] = 0x00;
  data[1] = ena;
  data[2] = dir;
  data[3] = (uint8_t)(pulse_target);
  data[4] = (uint8_t)(pulse_target >> 8);
  data[5] = (uint8_t)(pulse_target >> 16);
  data[6] = (uint8_t)(pps);
  data[7] = (uint8_t)(pps >> 8);
  can_error = CAN.sendMsgBuf(can_id, 0, 8, data) != CAN_OK;

  if (can_error)
  {
    // Read ELFG Register
    uint8_t eflg = mcpReadRegister(0x2D);
    if ((eflg & (1 << 5)) || // Bus-Off (TXBO)
        (eflg & (1 << 4)) || // TX Passive (TXEP)
        (eflg & (1 << 2)))   // TX Warning (TXWAR)
    {
      mcpReset(); // Reset only for transmission-related errors
    }
  }

}
