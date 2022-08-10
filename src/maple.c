/**
 * Pop'n'Music controller
 * Dreamcast Maple Bus Transiever example for Raspberry Pi Pico (RP2040)
 * (C) Charlie Cole 2021
 *
 * Modified by Mackie Kannard-Smith 2022
 * SSD1306 library by James Hughes (JamesH65)
 *
 * Dreamcast controller connector pin 1 (Maple Bus A) to GP11 (MAPLE_A)
 * Dreamcast controller connector pin 5 (Maple Bus B) to GP12 (MAPLE_B)
 * Dreamcast controller connector pins 3 (GND) and 4 (Sense) to GND
 * GPIO pins for buttons (uses internal pullups, switch to GND. See ButtonInfos in maple.h)
 *
 * Maple TX done completely in PIO. Sends start of packet, data and end of
 * packet. Fed by DMA so fire and forget.
 *
 * Maple RX done mostly in software on core 1. PIO just waits for transitions
 * and shifts in whenever data pins change. For maximum speed the RX state
 * machine is implemented in lookup table to process 4 transitions at a time
 */

#include "maple.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "format.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "maple.pio.h"
#include "menu.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "ssd1306.h"
#include "ssd1331.h"
#include "state_machine.h"

// Maple Bus Defines and Funcs

#define SHOULD_SEND 0  // Set to zero to sniff two devices sending signals to each other
#define SHOULD_PRINT 0 // Nice for debugging but can cause timing issues

// Board Variant
#define PICO 1
#define MAPLEPAD 0

// HKT-7700 (Standard Controller) or HKT-7300 (Arcade Stick) (see maple.h)
#if HKT7700
#define NUM_BUTTONS 9
#elif HKT7300
#define NUM_BUTTONS 11
#endif

// Purupuru Enable
#define ENABLE_RUMBLE 1

#define VMU 0
#define PURUPURU 1

#define PHASE_SIZE (BLOCK_SIZE / 4)
#define FLASH_WRITE_DELAY 16       // About quarter of a second if polling once a frame
#define FLASH_OFFSET (128 * 1024) // How far into Flash to store the memory card data. We only have around 75kB of code so assuming this will be fine

// ### TO-DO: Check for button combo in SendControllerStatus to page cycle on MaplePad
#if PICO
#define PAGE_BUTTON 21 // Pull GP21 low for Page Cycle. Avoid page cycling for ~10s after saving or copying VMU data to avoid data corruption
#elif MAPLEPAD
#define PAGE_BUTTON 20 // Dummy pin
#endif

#define PAGE_BUTTON_MASK 0x0608 // X, Y, and Start

#define CAL_MODE 20
#define OLED_PIN 22

#define MAPLE_A 11
#define MAPLE_B 12
#define PICO_PIN1_PIN_RX MAPLE_A
#define PICO_PIN5_PIN_RX MAPLE_B

#define ADDRESS_DREAMCAST 0
#define ADDRESS_CONTROLLER 0x20
#define ADDRESS_SUBPERIPHERAL0 0x01
#define ADDRESS_SUBPERIPHERAL1 0x02

#if ENABLE_RUMBLE
#define ADDRESS_CONTROLLER_AND_SUBS (ADDRESS_CONTROLLER | ADDRESS_SUBPERIPHERAL0 | ADDRESS_SUBPERIPHERAL1) // Determines which peripherals MaplePad reports
#else
#define ADDRESS_CONTROLLER_AND_SUBS (ADDRESS_CONTROLLER | ADDRESS_SUBPERIPHERAL0)
#endif

#define ADDRESS_PORT_MASK 0xC0
#define ADDRESS_PERIPHERAL_MASK (~ADDRESS_PORT_MASK)

#define TXPIO pio0
#define RXPIO pio1

#define SWAP4(x)              \
  do                          \
  {                           \
    x = __builtin_bswap32(x); \
  } while (0)

#define LCD_Width 48
#define LCD_Height 32
#define LCD_NumCols 6          // 48 / 8
#define LCDFramebufferSize 192 // (48 * 32) / 8
#define BPPacket 192           // Bytes Per Packet

const uint16_t color = 0xffff;

volatile uint16_t palette[] = {
    0xf800, // red
    0xfba0, // orange
    0xff80, // yellow
    0x7f80, // yellow-green
    0x0500, // green
    0x045f, // blue
    0x781f, // violet
    0x780d  // magenta
};

typedef enum ESendState_e
{
  SEND_NOTHING,
  SEND_CONTROLLER_INFO,
  SEND_CONTROLLER_STATUS,
  SEND_MEMORY_CARD_INFO,
  SEND_MEMORY_INFO,
  SEND_LCD_INFO,
  SEND_TIMER_INFO,
  SEND_PURUPURU_INFO,
  SEND_ACK,
  SEND_DATA,
  SEND_PURUPURU_DATA,
  SEND_CONDITION //
} ESendState;

enum ECommands
{
  CMD_RESPOND_FILE_ERROR = -5,
  CMD_RESPOND_SEND_AGAIN = -4,
  CMD_RESPOND_UNKNOWN_COMMAND = -3,
  CMD_RESPOND_FUNC_CODE_UNSUPPORTED = -2,
  CMD_NO_RESPONSE = -1,
  CMD_REQUEST_DEVICE_INFO = 1,
  CMD_REQUEST_EXTENDED_DEVICE_INFO,
  CMD_RESET_DEVICE,
  CMD_SHUTDOWN_DEVICE,
  CMD_RESPOND_DEVICE_INFO,
  CMD_RESPOND_EXTENDED_DEVICE_INFO,
  CMD_RESPOND_COMMAND_ACK,
  CMD_RESPOND_DATA_TRANSFER,
  CMD_GET_CONDITION,
  CMD_GET_MEDIA_INFO,
  CMD_BLOCK_READ,
  CMD_BLOCK_WRITE,
  CMD_BLOCK_COMPLETE_WRITE,
  CMD_SET_CONDITION //
};

enum EFunction
{
  FUNC_CONTROLLER = 1,  // FT0
  FUNC_MEMORY_CARD = 2, // FT1
  FUNC_LCD = 4,         // FT2
  FUNC_TIMER = 8,       // FT3
  FUNC_VIBRATION = 256  // FT8
};

// Buffers
static uint8_t RecieveBuffer[4096] __attribute__((aligned(4))); // Ring buffer for reading packets
static uint8_t Packet[1024 + 8] __attribute__((aligned(4)));    // Temp buffer for consuming packets (could remove)

static FControllerPacket ControllerPacket;                  // Send buffer for controller status packet (pre-built for speed)
static FInfoPacket InfoPacket;                              // Send buffer for controller info packet (pre-built for speed)
static FInfoPacket SubPeripheral0InfoPacket;                // Send buffer for memory card info packet (pre-built for speed)
static FInfoPacket SubPeripheral1InfoPacket;                // Send buffer for memory card info packet (pre-built for speed)
static FMemoryInfoPacket MemoryInfoPacket;                  // Send buffer for memory card info packet (pre-built for speed)
static FLCDInfoPacket LCDInfoPacket;                        // Send buffer for LCD info packet (pre-built for speed)
static FTimerInfoPacket TimerInfoPacket;                    // Send buffer for Timer info packet (pre-built for speed)
static FPuruPuruInfoPacket PuruPuruInfoPacket;              // Send buffer for PuruPuru info packet (pre-built for speed)
static FPuruPuruConditionPacket PuruPuruConditionPacket;    // Send buffer for PuruPuru condition packet (pre-built for speed)
static FACKPacket ACKPacket;                                // Send buffer for ACK packet (pre-built for speed)
static FBlockReadResponsePacket DataPacket;                 // Send buffer for Data packet (pre-built for speed)
static FPuruPuruBlockReadResponsePacket PuruPuruDataPacket; // Send buffer for PuruPuru packet (pre-built for speed)

static ESendState NextPacketSend = SEND_NOTHING;
static uint OriginalControllerCRC = 0;
static uint OriginalReadBlockResponseCRC = 0;
static uint TXDMAChannel = 0;

// Memory Card
static uint8_t MemoryCard[128 * 1024];
static uint SectorDirty = 0;
static uint SendBlockAddress = ~0u;
static uint MessagesSinceWrite = FLASH_WRITE_DELAY;
static uint32_t CurrentPage = 1;
volatile bool PageCycle = false;
volatile bool VMUCycle = false;
uint32_t lastPress = 0;

// LCD
static const uint8_t NumWrites = LCDFramebufferSize / BPPacket;
static uint8_t LCDFramebuffer[LCDFramebufferSize] = {0};
volatile bool LCDUpdated = false;
static volatile bool colorOLED = true; // True = SSD1331, false = SSD1306

// Purupuru
static bool purupuruUpdated = false;
static uint32_t prevpurupuru_cond = 0;
static uint32_t purupuru_cond = 0;
static uint8_t ctrl = 0;
static uint8_t power = 0;
static uint8_t freq = 0;
static uint8_t inc = 0;

static uint32_t vibeFreqCount = 0;

static uint8_t AST[4] = {0, 0, 0x13, 0x13}; // Vibration auto-stop time setting. Default is 5s
static uint32_t AST_timestamp = 0;

uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) { 

  if(x < in_min)
    return out_min;
  else if (x > in_max)
    return out_max;
  else return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
  
}

uint32_t map_uint32(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) { 
  if(x < in_min)
    return out_min;
  else if (x > in_max)
    return out_max;
  else return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
  
}

uint8_t min(uint8_t x, uint8_t y) { return x <= y ? x : y; }

uint8_t max(uint8_t x, uint8_t y) { return x >= y ? x : y; }

static uint StickConfig[14] = {0}; // Joystick configuration values

uint CalcCRC(const uint *Words, uint NumWords)
{
  uint XOR_Checksum = 0;
  for (uint i = 0; i < NumWords; i++)
  {
    XOR_Checksum ^= *(Words++);
  }
  XOR_Checksum ^= (XOR_Checksum << 16);
  XOR_Checksum ^= (XOR_Checksum << 8);
  return XOR_Checksum;
}

void BuildACKPacket()
{
  ACKPacket.BitPairsMinus1 = (sizeof(ACKPacket) - 7) * 4 - 1;

  ACKPacket.Header.Command = CMD_RESPOND_COMMAND_ACK;
  ACKPacket.Header.Destination = ADDRESS_DREAMCAST;
  ACKPacket.Header.Origin = ADDRESS_SUBPERIPHERAL0;
  ACKPacket.Header.NumWords = 0;

  ACKPacket.CRC = CalcCRC((uint *)&ACKPacket.Header, sizeof(ACKPacket) / sizeof(uint) - 2);
}

void BuildInfoPacket()
{
  InfoPacket.BitPairsMinus1 = (sizeof(InfoPacket) - 7) * 4 - 1;

  InfoPacket.Header.Command = CMD_RESPOND_DEVICE_INFO;
  InfoPacket.Header.Destination = ADDRESS_DREAMCAST;
  InfoPacket.Header.Origin = ADDRESS_CONTROLLER_AND_SUBS;
  InfoPacket.Header.NumWords = sizeof(InfoPacket.Info) / sizeof(uint);

#if HKT7700
  InfoPacket.Info.Func = __builtin_bswap32(FUNC_CONTROLLER);
  InfoPacket.Info.FuncData[0] = __builtin_bswap32(0x000f06fe); // What buttons it supports
  InfoPacket.Info.FuncData[1] = 0;
  InfoPacket.Info.FuncData[2] = 0;
  InfoPacket.Info.AreaCode = -1;
  InfoPacket.Info.ConnectorDirection = 0;
  strncpy(InfoPacket.Info.ProductName, "MaplePad                     ", sizeof(InfoPacket.Info.ProductName));

  strncpy(InfoPacket.Info.ProductLicense,
          // NOT REALLY! Don't sue me Sega!
          "Produced By or Under License From SEGA ENTERPRISES,LTD.    ", sizeof(InfoPacket.Info.ProductLicense));
  InfoPacket.Info.StandbyPower = 430;
  InfoPacket.Info.MaxPower = 500;
#elif HKT7300
  InfoPacket.Info.Func = __builtin_bswap32(FUNC_CONTROLLER);
  InfoPacket.Info.FuncData[0] = __builtin_bswap32(0x000007FF); // What buttons it supports
  InfoPacket.Info.FuncData[1] = 0;
  InfoPacket.Info.FuncData[2] = 0;
  InfoPacket.Info.AreaCode = -1;
  InfoPacket.Info.ConnectorDirection = 0;
  strncpy(InfoPacket.Info.ProductName, "Arcade Stick                 ", sizeof(InfoPacket.Info.ProductName));

  strncpy(InfoPacket.Info.ProductLicense,
          // NOT REALLY! Don't sue me Sega!
          "Produced By or Under License From SEGA ENTERPRISES,LTD.    ", sizeof(InfoPacket.Info.ProductLicense));
  InfoPacket.Info.StandbyPower = 30;
  InfoPacket.Info.MaxPower = 40;
#endif

  InfoPacket.CRC = CalcCRC((uint *)&InfoPacket.Header, sizeof(InfoPacket) / sizeof(uint) - 2);
}

void BuildSubPeripheral0InfoPacket() // Visual Memory Unit
{
  SubPeripheral0InfoPacket.BitPairsMinus1 = (sizeof(SubPeripheral0InfoPacket) - 7) * 4 - 1;

  SubPeripheral0InfoPacket.Header.Command = CMD_RESPOND_DEVICE_INFO;
  SubPeripheral0InfoPacket.Header.Destination = ADDRESS_DREAMCAST;
  SubPeripheral0InfoPacket.Header.Origin = ADDRESS_SUBPERIPHERAL0;
  SubPeripheral0InfoPacket.Header.NumWords = sizeof(SubPeripheral0InfoPacket.Info) / sizeof(uint);

  SubPeripheral0InfoPacket.Info.Func = __builtin_bswap32(0x0E);              // Function Types (up to 3). Note: Higher index in FuncData means
                                                                             // higher priority on DC subperipheral
  SubPeripheral0InfoPacket.Info.FuncData[0] = __builtin_bswap32(0x7E7E3F40); // Function Definition Block for Function Type 3 (Timer)
  SubPeripheral0InfoPacket.Info.FuncData[1] = __builtin_bswap32(0x00051000); // Function Definition Block for Function Type 2 (LCD)
  SubPeripheral0InfoPacket.Info.FuncData[2] = __builtin_bswap32(0x000f4100); // Function Definition Block for Function Type 1 (Storage)
  SubPeripheral0InfoPacket.Info.AreaCode = -1;
  SubPeripheral0InfoPacket.Info.ConnectorDirection = 0;
  strncpy(SubPeripheral0InfoPacket.Info.ProductName, "Visual Memory                ", sizeof(SubPeripheral0InfoPacket.Info.ProductName));
  strncpy(SubPeripheral0InfoPacket.Info.ProductLicense,
          // NOT REALLY! Don't sue me Sega!
          "Produced By or Under License From SEGA ENTERPRISES,LTD.    ", sizeof(SubPeripheral0InfoPacket.Info.ProductLicense));
  SubPeripheral0InfoPacket.Info.StandbyPower = 100;
  SubPeripheral0InfoPacket.Info.MaxPower = 130;

  SubPeripheral0InfoPacket.CRC = CalcCRC((uint *)&SubPeripheral0InfoPacket.Header, sizeof(SubPeripheral0InfoPacket) / sizeof(uint) - 2);
}

void BuildSubPeripheral1InfoPacket() // PuruPuru Pack
{
  SubPeripheral1InfoPacket.BitPairsMinus1 = (sizeof(SubPeripheral1InfoPacket) - 7) * 4 - 1;

  SubPeripheral1InfoPacket.Header.Command = CMD_RESPOND_DEVICE_INFO;
  SubPeripheral1InfoPacket.Header.Destination = ADDRESS_DREAMCAST;
  SubPeripheral1InfoPacket.Header.Origin = ADDRESS_SUBPERIPHERAL1;
  SubPeripheral1InfoPacket.Header.NumWords = sizeof(SubPeripheral1InfoPacket.Info) / sizeof(uint);

  SubPeripheral1InfoPacket.Info.Func = __builtin_bswap32(0x100);             // Function Types (up to 3). Note: Higher index in FuncData means
                                                                             // higher priority on DC subperipheral
  SubPeripheral1InfoPacket.Info.FuncData[0] = __builtin_bswap32(0x04500000); // Function Definition Block for Function Type 1 (Vibration)
  SubPeripheral1InfoPacket.Info.FuncData[1] = 0;
  SubPeripheral1InfoPacket.Info.FuncData[2] = 0;
  SubPeripheral1InfoPacket.Info.AreaCode = -1;
  SubPeripheral1InfoPacket.Info.ConnectorDirection = 0;
  strncpy(SubPeripheral1InfoPacket.Info.ProductName, "Puru Puru Pack               ", sizeof(SubPeripheral1InfoPacket.Info.ProductName));
  strncpy(SubPeripheral1InfoPacket.Info.ProductLicense,
          // NOT REALLY! Don't sue me Sega!
          "Produced By or Under License From SEGA ENTERPRISES,LTD.    ", sizeof(SubPeripheral1InfoPacket.Info.ProductLicense));
  SubPeripheral1InfoPacket.Info.StandbyPower = 200;
  SubPeripheral1InfoPacket.Info.MaxPower = 1600;

  SubPeripheral1InfoPacket.CRC = CalcCRC((uint *)&SubPeripheral1InfoPacket.Header, sizeof(SubPeripheral1InfoPacket) / sizeof(uint) - 2);
}

void BuildMemoryInfoPacket()
{
  MemoryInfoPacket.BitPairsMinus1 = (sizeof(MemoryInfoPacket) - 7) * 4 - 1;

  MemoryInfoPacket.Header.Command = CMD_RESPOND_DATA_TRANSFER;
  MemoryInfoPacket.Header.Destination = ADDRESS_DREAMCAST;
  MemoryInfoPacket.Header.Origin = ADDRESS_SUBPERIPHERAL0;
  MemoryInfoPacket.Header.NumWords = sizeof(MemoryInfoPacket.Info) / sizeof(uint);

  MemoryInfoPacket.Info.Func = __builtin_bswap32(FUNC_MEMORY_CARD);
  // This seems to be what emulators return but some values seem a bit weird
  // TODO: Sniff the communication with a real VMU
  MemoryInfoPacket.Info.TotalSize = CARD_BLOCKS - 1;
  MemoryInfoPacket.Info.ParitionNumber = 0;
  MemoryInfoPacket.Info.SystemArea = ROOT_BLOCK; // Seems like this should be root block instead of "system
                                                 // area"
  MemoryInfoPacket.Info.FATArea = FAT_BLOCK;
  MemoryInfoPacket.Info.NumFATBlocks = NUM_FAT_BLOCKS;
  MemoryInfoPacket.Info.FileInfoArea = DIRECTORY_BLOCK;
  MemoryInfoPacket.Info.NumInfoBlocks = NUM_DIRECTORY_BLOCKS; // Grows downwards?
  MemoryInfoPacket.Info.VolumeIcon = 0;
  MemoryInfoPacket.Info.Reserved = 0;
  MemoryInfoPacket.Info.SaveArea = SAVE_BLOCK;
  MemoryInfoPacket.Info.NumSaveBlocks = NUM_SAVE_BLOCKS; // Grows downwards? Shouldn't it to be 200?
  MemoryInfoPacket.Info.Reserved32 = 0;

  MemoryInfoPacket.CRC = CalcCRC((uint *)&MemoryInfoPacket.Header, sizeof(MemoryInfoPacket) / sizeof(uint) - 2);
}

void BuildLCDInfoPacket()
{
  LCDInfoPacket.BitPairsMinus1 = (sizeof(LCDInfoPacket) - 7) * 4 - 1;

  LCDInfoPacket.Header.Command = CMD_RESPOND_DATA_TRANSFER;
  LCDInfoPacket.Header.Destination = ADDRESS_DREAMCAST;
  LCDInfoPacket.Header.Origin = ADDRESS_SUBPERIPHERAL0;
  LCDInfoPacket.Header.NumWords = sizeof(LCDInfoPacket.Info) / sizeof(uint);

  LCDInfoPacket.Info.Func = __builtin_bswap32(FUNC_LCD);

  LCDInfoPacket.Info.dX = LCD_Width - 1;  // 48 dots wide (dX + 1)
  LCDInfoPacket.Info.dY = LCD_Height - 1; // 32 dots tall (dY + 1)
  LCDInfoPacket.Info.GradContrast = 0x10; // Gradation = 1 bit/dot, Contrast = 0 (disabled)
  LCDInfoPacket.Info.Reserved = 0;

  LCDInfoPacket.CRC = CalcCRC((uint *)&LCDInfoPacket.Header, sizeof(LCDInfoPacket) / sizeof(uint) - 2);
}

void BuildTimerInfoPacket()
{
  LCDInfoPacket.BitPairsMinus1 = (sizeof(LCDInfoPacket) - 7) * 4 - 1;

  LCDInfoPacket.Header.Command = CMD_RESPOND_DATA_TRANSFER;
  LCDInfoPacket.Header.Destination = ADDRESS_DREAMCAST;
  LCDInfoPacket.Header.Origin = ADDRESS_SUBPERIPHERAL0;
  LCDInfoPacket.Header.NumWords = sizeof(LCDInfoPacket.Info) / sizeof(uint);

  LCDInfoPacket.Info.Func = __builtin_bswap32(FUNC_TIMER);

  LCDInfoPacket.Info.dX = LCD_Width - 1;  // 48 dots wide (dX + 1)
  LCDInfoPacket.Info.dY = LCD_Height - 1; // 32 dots tall (dY + 1)
  LCDInfoPacket.Info.GradContrast = 0x10; // Gradation = 1 bit/dot, Contrast = 0 (disabled)
  LCDInfoPacket.Info.Reserved = 0;

  LCDInfoPacket.CRC = CalcCRC((uint *)&LCDInfoPacket.Header, sizeof(LCDInfoPacket) / sizeof(uint) - 2);
}

void BuildPuruPuruInfoPacket()
{
  PuruPuruInfoPacket.BitPairsMinus1 = (sizeof(PuruPuruInfoPacket) - 7) * 4 - 1;

  PuruPuruInfoPacket.Header.Command = CMD_RESPOND_DATA_TRANSFER;
  PuruPuruInfoPacket.Header.Destination = ADDRESS_DREAMCAST;
  PuruPuruInfoPacket.Header.Origin = ADDRESS_SUBPERIPHERAL1;
  PuruPuruInfoPacket.Header.NumWords = sizeof(PuruPuruInfoPacket.Info) / sizeof(uint);

  PuruPuruInfoPacket.Info.Func = __builtin_bswap32(FUNC_VIBRATION);

  PuruPuruInfoPacket.Info.VSet0 = 0x10;
  // PuruPuruInfoPacket.Info.Vset1 = 0xC0;
  PuruPuruInfoPacket.Info.Vset1 = 0x80; // disable continuous vibration for now
  PuruPuruInfoPacket.Info.FMin = 0x07;
  PuruPuruInfoPacket.Info.FMin = 0x3B;

  // PuruPuruInfoPacket.Info.VSet0 = 0x10;
  // PuruPuruInfoPacket.Info.Vset1 = 0x1f;
  // PuruPuruInfoPacket.Info.FMin = 0x00;
  // PuruPuruInfoPacket.Info.FMin = 0x00;

  PuruPuruInfoPacket.CRC = CalcCRC((uint *)&PuruPuruInfoPacket.Header, sizeof(PuruPuruInfoPacket) / sizeof(uint) - 2);
}

void BuildPuruPuruConditionPacket()
{
  PuruPuruConditionPacket.BitPairsMinus1 = (sizeof(PuruPuruConditionPacket) - 7) * 4 - 1;

  PuruPuruConditionPacket.Header.Command = CMD_RESPOND_DATA_TRANSFER;
  PuruPuruConditionPacket.Header.Destination = ADDRESS_DREAMCAST;
  PuruPuruConditionPacket.Header.Origin = ADDRESS_SUBPERIPHERAL1;
  PuruPuruConditionPacket.Header.NumWords = sizeof(PuruPuruConditionPacket.Condition) / sizeof(uint);

  PuruPuruConditionPacket.Condition.Func = __builtin_bswap32(FUNC_VIBRATION);

  PuruPuruConditionPacket.Condition.VSource = 0x00;
  PuruPuruConditionPacket.Condition.Power = 0x00;
  PuruPuruConditionPacket.Condition.Freq = 0x00;
  PuruPuruConditionPacket.Condition.Inc = 0x00;

  PuruPuruConditionPacket.CRC = CalcCRC((uint *)&PuruPuruConditionPacket.Header, sizeof(PuruPuruConditionPacket) / sizeof(uint) - 2);
}

void BuildControllerPacket()
{
  ControllerPacket.BitPairsMinus1 = (sizeof(ControllerPacket) - 7) * 4 - 1;

  ControllerPacket.Header.Command = CMD_RESPOND_DATA_TRANSFER;
  ControllerPacket.Header.Destination = ADDRESS_DREAMCAST;
  ControllerPacket.Header.Origin = ADDRESS_CONTROLLER_AND_SUBS;
  ControllerPacket.Header.NumWords = sizeof(ControllerPacket.Controller) / sizeof(uint);

  ControllerPacket.Controller.Condition = __builtin_bswap32(FUNC_CONTROLLER);
  ControllerPacket.Controller.Buttons = 0;
  ControllerPacket.Controller.LeftTrigger = 0;
  ControllerPacket.Controller.RightTrigger = 0;
  ControllerPacket.Controller.JoyX = 0x80;
  ControllerPacket.Controller.JoyY = 0x80;
  ControllerPacket.Controller.JoyX2 = 0x80;
  ControllerPacket.Controller.JoyY2 = 0x80;

  OriginalControllerCRC = CalcCRC((uint *)&ControllerPacket.Header, sizeof(ControllerPacket) / sizeof(uint) - 2);
}

void BuildDataPacket()
{
  DataPacket.BitPairsMinus1 = (sizeof(DataPacket) - 7) * 4 - 1;

  DataPacket.Header.Command = CMD_RESPOND_DATA_TRANSFER;
  DataPacket.Header.Destination = ADDRESS_DREAMCAST;
  DataPacket.Header.Origin = ADDRESS_SUBPERIPHERAL0;
  DataPacket.Header.NumWords = sizeof(DataPacket.BlockRead) / sizeof(uint);

  DataPacket.BlockRead.Func = __builtin_bswap32(FUNC_MEMORY_CARD);
  DataPacket.BlockRead.Address = 0;
  memset(DataPacket.BlockRead.Data, 0, sizeof(DataPacket.BlockRead.Data));

  OriginalReadBlockResponseCRC = CalcCRC((uint *)&DataPacket.Header, sizeof(DataPacket) / sizeof(uint) - 2);
}

void BuildPuruPuruBlockReadPacket()
{
  PuruPuruDataPacket.BitPairsMinus1 = (sizeof(PuruPuruDataPacket) - 7) * 4 - 1;

  PuruPuruDataPacket.Header.Command = CMD_RESPOND_DATA_TRANSFER;
  PuruPuruDataPacket.Header.Destination = ADDRESS_DREAMCAST;
  PuruPuruDataPacket.Header.Origin = ADDRESS_SUBPERIPHERAL1;
  PuruPuruDataPacket.Header.NumWords = sizeof(PuruPuruDataPacket.PuruPuruBlockRead) / sizeof(uint);

  PuruPuruDataPacket.PuruPuruBlockRead.Func = __builtin_bswap32(FUNC_VIBRATION);
  PuruPuruDataPacket.PuruPuruBlockRead.Address = 0;
  memset(PuruPuruDataPacket.PuruPuruBlockRead.Data, 0, sizeof(PuruPuruDataPacket.PuruPuruBlockRead.Data));

  OriginalReadBlockResponseCRC = CalcCRC((uint *)&PuruPuruDataPacket.Header, sizeof(PuruPuruDataPacket) / sizeof(uint) - 2);
}

int SendPacket(const uint *Words, uint NumWords)
{
  // Correct the port number. Doesn't change CRC as same on both Origin and Destination
  PacketHeader *Header = (PacketHeader *)(Words + 1);
  Header->Origin = (Header->Origin & ADDRESS_PERIPHERAL_MASK) | (((PacketHeader *)Packet)->Origin & ADDRESS_PORT_MASK);
  Header->Destination = (Header->Destination & ADDRESS_PERIPHERAL_MASK) | (((PacketHeader *)Packet)->Origin & ADDRESS_PORT_MASK);

  dma_channel_set_read_addr(TXDMAChannel, Words, false);
  dma_channel_set_trans_count(TXDMAChannel, NumWords, true);
}

void SendControllerStatus()
{
  uint Buttons = 0x0000FFFF;

  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    if (!gpio_get(ButtonInfos[i].InputIO))
    {
      Buttons &= ~ButtonInfos[i].DCButtonMask;
    }
  }

#if MAPLEPAD
  // placeholder
#endif

  if ((Buttons & PAGE_BUTTON_MASK) == 0 && !PageCycle)
  {
    uint32_t pressTime = to_ms_since_boot(get_absolute_time());
    if ((pressTime - lastPress) >= 500)
      {
        if (CurrentPage == 8)
          CurrentPage = 1;
        else
          CurrentPage++;
        PageCycle = true;
        lastPress = pressTime;
      }
  }

  ControllerPacket.Controller.Buttons = Buttons;

  // StickConfig[0] 	// xCenter
  // StickConfig[1] 	// xMin
  // StickConfig[2] 	// xMax
  // StickConfig[3]	  // yCenter
  // StickConfig[4]	  // yMin
  // StickConfig[5]	  // yMax
  // StickConfig[6]	  // lMax
  // StickConfig[7]	  // lMin
  // StickConfig[8]	  // rMax
  // StickConfig[9] 	// rMin

#if HKT7700 // Only HKT-7700 has analog stick and triggers

  adc_select_input(0);
  uint8_t xRead = adc_read() >> 4;
  if (xRead > (StickConfig[0] - 0x0F) && xRead < (StickConfig[0] + 0x0F)) // deadzone
    ControllerPacket.Controller.JoyX = 0x80;
  else if (xRead < StickConfig[0])
    ControllerPacket.Controller.JoyX = map(xRead, StickConfig[1] - 0x04, StickConfig[0] - 0x0F, 0x00, 0x7F);
  else if (xRead > StickConfig[0])
    ControllerPacket.Controller.JoyX = map(xRead, StickConfig[0] + 0x0F, StickConfig[2] + 0x04, 0x81, 0xFF);

  adc_select_input(1);
  uint8_t yRead = adc_read() >> 4;
  if (yRead > (StickConfig[3] - 0x0F) && yRead < (StickConfig[3] + 0x0F)) // deadzone
    ControllerPacket.Controller.JoyY = 0x80;
  else if (yRead < StickConfig[3])
    ControllerPacket.Controller.JoyY = map(yRead, StickConfig[4] - 0x04, StickConfig[3] - 0x0F, 0x00, 0x7F);
  else if (yRead > StickConfig[3])
    ControllerPacket.Controller.JoyY = map(yRead, StickConfig[3] + 0x0F, StickConfig[5] + 0x04, 0x81, 0xFF);

  adc_select_input(2);
  uint8_t lRead = adc_read() >> 4;
  if (StickConfig[12]) // invertL
  {                                      
    if (lRead > (StickConfig[7] - 0x08)) // deadzone
      ControllerPacket.Controller.LeftTrigger = 0x00;
    else if (lRead >= (StickConfig[6] - 0x0F) > StickConfig[6] ? 0x00 : (StickConfig[6] - 0x0F))
      ControllerPacket.Controller.LeftTrigger = map(lRead, (StickConfig[6] - 0x04) > StickConfig[6] ? 0x00 : (StickConfig[6] - 0x04), (StickConfig[7] + 0x04) < StickConfig[7] ? 0xFF : (StickConfig[7] + 0x04), 0xFF, 0x01);
  }
  else
  {
    if (lRead < (StickConfig[6] + 0x08)) // deadzone
      ControllerPacket.Controller.LeftTrigger = 0x00;
    else if (lRead <= (StickConfig[7] + 0x0F) < StickConfig[7] ? 0xFF : (StickConfig[7] + 0x0F))
      ControllerPacket.Controller.LeftTrigger = map(lRead, (StickConfig[6] - 0x04) > StickConfig[6] ? 0x00 : (StickConfig[6] - 0x04), (StickConfig[7] + 0x04) < StickConfig[7] ? 0xFF : (StickConfig[7] + 0x04), 0x01, 0xFF);
  }

  adc_select_input(3);
  uint8_t rRead = adc_read() >> 4;
  if (StickConfig[13]) // invertR
  {                                      
    if (rRead > (StickConfig[9] - 0x08)) // deadzone
      ControllerPacket.Controller.RightTrigger = 0x00;
    else if (rRead >= (StickConfig[8] - 0x0F) > StickConfig[8] ? 0x00 : (StickConfig[8] - 0x0F))
      ControllerPacket.Controller.RightTrigger = map(rRead, (StickConfig[8] - 0x04) > StickConfig[8] ? 0x00 : (StickConfig[8] - 0x04), (StickConfig[9] + 0x04) < StickConfig[9] ? 0xFF : (StickConfig[9] + 0x04), 0xFF, 0x01);
  }
  else
  {
    if (rRead < (StickConfig[8] + 0x08)) // deadzone
      ControllerPacket.Controller.RightTrigger = 0x00;
    else if (rRead <= (StickConfig[9] + 0x0F) < StickConfig[9] ? 0xFF : (StickConfig[9] + 0x0F))
      ControllerPacket.Controller.RightTrigger = map(rRead, (StickConfig[8] - 0x04) > StickConfig[8] ? 0x00 : (StickConfig[8] - 0x04), (StickConfig[9] + 0x04) < StickConfig[9] ? 0xFF : (StickConfig[9] + 0x04), 0x01, 0xFF);
  }

#endif

  ControllerPacket.CRC = CalcCRC((uint *)&ControllerPacket.Header, sizeof(ControllerPacket) / sizeof(uint) - 2);

  SendPacket((uint *)&ControllerPacket, sizeof(ControllerPacket) / sizeof(uint));
}

void SendBlockReadResponsePacket(uint PuruPuru)
{
  uint Partition = (SendBlockAddress >> 24) & 0xFF;
  uint Phase = (SendBlockAddress >> 16) & 0xFF;
  uint Block = SendBlockAddress & 0xFF; // Emulators also seem to ignore top bits for a read

  assert(Phase == 0); // Note: Phase is analogous to the VN parameter in a vibration AST block R/W.
  uint MemoryOffset = Block * BLOCK_SIZE + Phase * PHASE_SIZE;

  if (PuruPuru)
  { // PuruPuru AST Block Read
    PuruPuruDataPacket.PuruPuruBlockRead.Address = SendBlockAddress;
    memcpy(PuruPuruDataPacket.PuruPuruBlockRead.Data, &AST[0], sizeof(PuruPuruDataPacket.PuruPuruBlockRead.Data));
    uint CRC = CalcCRC(&PuruPuruDataPacket.PuruPuruBlockRead.Address, 1 + (sizeof(PuruPuruDataPacket.PuruPuruBlockRead.Data) / sizeof(uint)));
    PuruPuruDataPacket.CRC = CRC ^ OriginalReadBlockResponseCRC;

    SendBlockAddress = ~0u;

    SendPacket((uint *)&PuruPuruDataPacket, sizeof(PuruPuruDataPacket) / sizeof(uint));
  }
  else
  { // Memory Card Block Read
    DataPacket.BlockRead.Address = SendBlockAddress;
    memcpy(DataPacket.BlockRead.Data, &MemoryCard[MemoryOffset], sizeof(DataPacket.BlockRead.Data));
    uint CRC = CalcCRC(&DataPacket.BlockRead.Address, 1 + (sizeof(DataPacket.BlockRead.Data) / sizeof(uint)));
    DataPacket.CRC = CRC ^ OriginalReadBlockResponseCRC;

    SendBlockAddress = ~0u;

    SendPacket((uint *)&DataPacket, sizeof(DataPacket) / sizeof(uint));
  }
}

void BlockRead(uint Address, uint PuruPuru)
{
  assert(SendBlockAddress == ~0u); // No send pending
  SendBlockAddress = Address;
  if (PuruPuru)
    NextPacketSend = SEND_PURUPURU_DATA;
  else
    NextPacketSend = SEND_DATA;
}

void BlockWrite(uint Address, uint *Data, uint NumWords)
{
  uint Partition = (Address >> 24) & 0xFF;
  uint Phase = (Address >> 16) & 0xFF;
  uint Block = Address & 0xFFFF;

  assert(NumWords * sizeof(uint) == PHASE_SIZE);

  uint MemoryOffset = Block * BLOCK_SIZE + Phase * PHASE_SIZE;
  memcpy(&MemoryCard[MemoryOffset], Data, PHASE_SIZE);
  SectorDirty |= 1u << (MemoryOffset / FLASH_SECTOR_SIZE);
  MessagesSinceWrite = 0;

  NextPacketSend = SEND_ACK;
}

void LCDWrite(uint Address, uint *Data, uint NumWords, uint BlockNum)
{
  assert(NumWords * sizeof(uint) == (LCDFramebufferSize / NumWrites));

  if (BlockNum == 0x10) // 128x64 Test Mode
  {
    memcpy(&LCDFramebuffer[512], Data, NumWords * sizeof(uint));
  }
  else if (BlockNum == 0x00)
  { // Normal mode
    memcpy(LCDFramebuffer, Data, NumWords * sizeof(uint));
  }

  LCDUpdated = true;

  NextPacketSend = SEND_ACK;
}

void PuruPuruWrite(uint Address, uint *Data, uint NumWords)
{
  memcpy(AST, Data, NumWords * sizeof(uint));
  NextPacketSend = SEND_ACK;
}

void BlockCompleteWrite(uint Address)
{
  uint Partition = (Address >> 24) & 0xFF;
  uint Phase = (Address >> 16) & 0xFF;
  uint Block = Address & 0xFFFF;

  assert(Phase == 4);

  uint MemoryOffset = Block * BLOCK_SIZE;
  SectorDirty |= 1u << (MemoryOffset / FLASH_SECTOR_SIZE);
  MessagesSinceWrite = 0;

  NextPacketSend = SEND_ACK;
}

bool ConsumePacket(uint Size)
{
  if ((Size & 3) == 1) // Even number of words + CRC
  {
    Size--; // Drop CRC byte
    if (Size > 0)
    {
      PacketHeader *Header = (PacketHeader *)Packet;
      uint *PacketData = (uint *)(Header + 1);
      if (Size == (Header->NumWords + 1) * 4)
      {
        // Mask off port number
        Header->Destination &= ADDRESS_PERIPHERAL_MASK;
        Header->Origin &= ADDRESS_PERIPHERAL_MASK;

        // If it's for us or we've sent something and want to check it
        if (Header->Destination == ADDRESS_CONTROLLER) // || (Header->Destination == ADDRESS_DREAMCAST && Header->Origin == ADDRESS_CONTROLLER_AND_SUBS))
        {
          switch (Header->Command)
          {
          case CMD_REQUEST_DEVICE_INFO:
          {
            NextPacketSend = SEND_CONTROLLER_INFO;
            return true;
          }
          case CMD_GET_CONDITION:
          {
            if (Header->NumWords >= 1 && *PacketData == __builtin_bswap32(FUNC_CONTROLLER))
            {
              NextPacketSend = SEND_CONTROLLER_STATUS;
              return true;
            }
            break;
          }
          case CMD_RESPOND_DEVICE_INFO:
          {
            PacketDeviceInfo *DeviceInfo = (PacketDeviceInfo *)(Header + 1);
            if (Header->NumWords * sizeof(uint) == sizeof(PacketDeviceInfo))
            {
              SWAP4(DeviceInfo->Func);
              SWAP4(DeviceInfo->FuncData[0]);
              SWAP4(DeviceInfo->FuncData[1]);
              SWAP4(DeviceInfo->FuncData[2]);
              return true;
            }
            break;
          }
          case CMD_RESPOND_DATA_TRANSFER:
          {
            PacketControllerCondition *ControllerCondition = (PacketControllerCondition *)(Header + 1);
            if (Header->NumWords * sizeof(uint) == sizeof(PacketControllerCondition))
            {
              SWAP4(ControllerCondition->Condition);
              if (ControllerCondition->Condition == FUNC_CONTROLLER)
              {
                return true;
              }
            }
            break;
          }
          }
        }
        // Subperipheral 0 (VMU)
        else if (Header->Destination == ADDRESS_SUBPERIPHERAL0) // || (Header->Destination == ADDRESS_DREAMCAST && Header->Origin == ADDRESS_SUBPERIPHERAL0))
        {
          switch (Header->Command)
          {
          case CMD_REQUEST_DEVICE_INFO:
          {
            NextPacketSend = SEND_MEMORY_CARD_INFO;
            return true;
          }
          case CMD_RESPOND_DEVICE_INFO:
          {
            PacketDeviceInfo *DeviceInfo = (PacketDeviceInfo *)(Header + 1);
            if (Header->NumWords * sizeof(uint) == sizeof(PacketDeviceInfo))
            {
              SWAP4(DeviceInfo->Func);
              SWAP4(DeviceInfo->FuncData[0]);
              SWAP4(DeviceInfo->FuncData[1]);
              SWAP4(DeviceInfo->FuncData[2]);
              return true;
            }
            break;
          }
          case CMD_GET_MEDIA_INFO:
          {
            if (Header->NumWords >= 2 && *PacketData == __builtin_bswap32(FUNC_MEMORY_CARD) && *(PacketData + 1) == 0)
            {
              NextPacketSend = SEND_MEMORY_INFO;
              return true;
            }
            else if (Header->NumWords >= 2 && *PacketData == __builtin_bswap32(FUNC_LCD) && *(PacketData + 1) == 0)
            {
              NextPacketSend = SEND_LCD_INFO;
              return true;
            }
            break;
          }
          case CMD_BLOCK_READ:
          {
            if (Header->NumWords >= 2)
            {
              assert(*PacketData == __builtin_bswap32(FUNC_MEMORY_CARD));
              BlockRead(__builtin_bswap32(*(PacketData + 1)), VMU);
              return true;
            }
            break;
          }
          case CMD_BLOCK_WRITE:
          {
            if (Header->NumWords >= 2 && *PacketData == __builtin_bswap32(FUNC_MEMORY_CARD))
            {
              BlockWrite(__builtin_bswap32(*(PacketData + 1)), PacketData + 2, Header->NumWords - 2);
              return true;
            }
            else if (Header->NumWords >= 2 && *PacketData == __builtin_bswap32(FUNC_LCD))
            {
              if (*(PacketData + 1) == __builtin_bswap32(0)) // Block 0
              {
                LCDWrite(__builtin_bswap32(*(PacketData + 1)), PacketData + 2, Header->NumWords - 2, 0);
                return true;
              }
              else if (*(PacketData + 1) == __builtin_bswap32(0x10))
              { // Block 1
                LCDWrite(__builtin_bswap32(*(PacketData + 1)), PacketData + 2, Header->NumWords - 2, 1);
                return true;
              }
            }
            break;
          }
          case CMD_BLOCK_COMPLETE_WRITE:
          {
            if (Header->NumWords >= 2)
            {
              assert(*PacketData == __builtin_bswap32(FUNC_MEMORY_CARD));
              BlockCompleteWrite(__builtin_bswap32(*(PacketData + 1)));
              return true;
            }
            break;
          }
          case CMD_RESPOND_DATA_TRANSFER:
          {
            return true;
            break;
          }
          case CMD_RESPOND_COMMAND_ACK:
          {
            if (Header->NumWords == 0)
              return true;
            break;
          }
          }
        }
        // Subperipheral 1 (Rumble)
        else if (Header->Destination == ADDRESS_SUBPERIPHERAL1) // || (Header->Destination == ADDRESS_DREAMCAST && Header->Origin == ADDRESS_SUBPERIPHERAL1))
        {
          switch (Header->Command)
          {
          case CMD_REQUEST_DEVICE_INFO:
          {
            NextPacketSend = SEND_PURUPURU_INFO;
            return true;
          }
          case CMD_RESPOND_DEVICE_INFO:
          {
            PacketDeviceInfo *DeviceInfo = (PacketDeviceInfo *)(Header + 1);
            if (Header->NumWords * sizeof(uint) == sizeof(PacketDeviceInfo))
            {
              SWAP4(DeviceInfo->Func);
              SWAP4(DeviceInfo->FuncData[0]);
              SWAP4(DeviceInfo->FuncData[1]);
              SWAP4(DeviceInfo->FuncData[2]);
              return true;
            }
            break;
          }
          case CMD_GET_MEDIA_INFO:
          {
            if (Header->NumWords >= 2 && *PacketData == __builtin_bswap32(FUNC_VIBRATION) && *(PacketData + 1) == 0)
            {
              NextPacketSend = SEND_PURUPURU_INFO;
              return true;
            }
            break;
          }
          case CMD_SET_CONDITION:
          {
            if (Header->NumWords >= 2)
            {
              assert(*PacketData == __builtin_bswap32(FUNC_VIBRATION));

              memcpy(&purupuru_cond, PacketData + 1, sizeof(purupuru_cond));

              ctrl = purupuru_cond & 0x000000ff;
              power = (purupuru_cond & 0x0000ff00) >> 8;
              freq = (purupuru_cond & 0x00ff0000) >> 16;
              inc = (purupuru_cond & 0xff000000) >> 24;

              if ((freq >= 0x07) && (freq <= 0x3B) && (ctrl & 0x10)) // check if frequency is in supported range
                purupuruUpdated = true;

              NextPacketSend = SEND_ACK;
              return true;
            }
            break;
          }
          case CMD_GET_CONDITION:
          {
            if (Header->NumWords >= 2)
            {
              assert(*PacketData == __builtin_bswap32(FUNC_VIBRATION));
              NextPacketSend = SEND_CONDITION;
              return true;
            }
            break;
          }
          case CMD_BLOCK_READ:
          {
            if (Header->NumWords >= 2)
            {
              assert(*PacketData == __builtin_bswap32(FUNC_VIBRATION));
              BlockRead(__builtin_bswap32(*(PacketData + 1)), PURUPURU);
              return true;
            }
            break;
          }
          case CMD_BLOCK_WRITE:
          {
            if (Header->NumWords >= 2 && *PacketData == __builtin_bswap32(FUNC_VIBRATION))
            {
              PuruPuruWrite(__builtin_bswap32(*(PacketData + 1)), PacketData + 2, Header->NumWords - 2);
              return true;
            }
            break;
          }
          case CMD_RESPOND_DATA_TRANSFER:
          {
            return true;
            break;
          }
          case CMD_RESPOND_COMMAND_ACK:
          {
            if (Header->NumWords == 0)
              return true;
            break;
          }
          }
        }
      }
    }
  }
  return false;
}

// *IMPORTANT* This function must be in RAM. Will be too slow if have to fetch
// code from flash
static void __not_in_flash_func(core1_entry)(void)
{
  uint State = 0;
  uint8_t Byte = 0;
  uint8_t XOR = 0;
  uint StartOfPacket = 0;
  uint Offset = 0;

  BuildStateMachineTables();

  multicore_fifo_push_blocking(0); // Tell core0 we're ready
  multicore_fifo_pop_blocking();   // Wait for core0 to acknowledge and start
                                   // RXPIO

  // Make sure we are ready to go	by flushing the FIFO
  while ((RXPIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB))) == 0)
  {
    pio_sm_get(RXPIO, 0);
  }

  while (true)
  {
    // Worst case we could have only 0.5us (~65 cycles) to process each byte if
    // want to keep up real time In practice we have around 4us on average so
    // this code is easily fast enough
    while ((RXPIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB))) != 0)
      ;
    const uint8_t Value = RXPIO->rxf[0];
    StateMachine M = Machine[State][Value];
    State = M.NewState;
    if (M.Reset)
    {
      Offset = StartOfPacket;
      Byte = 0;
      XOR = 0;
    }
    Byte |= SetBits[M.SetBitsIndex][0];
    if (M.Push)
    {
      RecieveBuffer[Offset & (sizeof(RecieveBuffer) - 1)] = Byte;
      XOR ^= Byte;
      Byte = SetBits[M.SetBitsIndex][1];
      Offset++;
    }
    if (M.End)
    {
      if (XOR == 0)
      {
        if (multicore_fifo_wready())
        {
          // multicore_fifo_push_blocking(Offset);  //Don't call as needs all be in RAM. Inlined below
          sio_hw->fifo_wr = Offset;
          __sev();
          StartOfPacket = ((Offset + 3) & ~3); // Align up for easier swizzling
        }
        else
        {
          //#if !SHOULD_PRINT // Core can be too slow due to printing
          panic("Packet processing core isn't fast enough :(\n");
          //#endif
        }
      }
    }
    if ((RXPIO->fstat & (1u << (PIO_FSTAT_RXFULL_LSB))) != 0)
    {
      // Should be a panic but the inlining of multicore_fifo_push_blocking caused it to fire
      // Weirdly after changing this to a printf it never gets called :/
      // Something to work out...
      printf("Probably overflowed. This code isn't fast enough :(\n");
    }
  }
}

void SetupButtons()
{
  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    gpio_init(ButtonInfos[i].InputIO);
    gpio_set_dir(ButtonInfos[i].InputIO, false);
    gpio_pull_up(ButtonInfos[i].InputIO);
  }
}

void SetupMapleTX()
{
  uint TXStateMachine = pio_claim_unused_sm(TXPIO, true);
  uint TXPIOOffset = pio_add_program(TXPIO, &maple_tx_program);
  maple_tx_program_init(TXPIO, TXStateMachine, TXPIOOffset, MAPLE_A, MAPLE_B, 3.0f);

  TXDMAChannel = dma_claim_unused_channel(true);
  dma_channel_config TXDMAConfig = dma_channel_get_default_config(TXDMAChannel);
  channel_config_set_read_increment(&TXDMAConfig, true);
  channel_config_set_write_increment(&TXDMAConfig, false);
  channel_config_set_transfer_data_size(&TXDMAConfig, DMA_SIZE_32);
  channel_config_set_dreq(&TXDMAConfig, pio_get_dreq(TXPIO, TXStateMachine, true));
  dma_channel_configure(TXDMAChannel, &TXDMAConfig,
                        &TXPIO->txf[TXStateMachine], // Destinatinon pointer
                        NULL,                        // Source pointer (will set when want to send)
                        0,                           // Number of transfers (will set when want to send)
                        false                        // Don't start yet
  );

  gpio_pull_up(MAPLE_A);
  gpio_pull_up(MAPLE_B);
}

void SetupMapleRX()
{
  uint RXPIOOffsets[3] = {pio_add_program(RXPIO, &maple_rx_triple1_program), pio_add_program(RXPIO, &maple_rx_triple2_program), pio_add_program(RXPIO, &maple_rx_triple3_program)};
  maple_rx_triple_program_init(RXPIO, RXPIOOffsets, PICO_PIN1_PIN_RX, PICO_PIN5_PIN_RX, 3.0f);

  // Make sure core1 is ready to say we are ready
  multicore_fifo_pop_blocking();
  multicore_fifo_push_blocking(0);

  pio_sm_set_enabled(RXPIO, 1, true);
  pio_sm_set_enabled(RXPIO, 2, true);
  pio_sm_set_enabled(RXPIO, 0, true);
}

void readFlash()
{
  memset(MemoryCard, 0, sizeof(MemoryCard));
  memcpy(MemoryCard, (uint8_t *)XIP_BASE + (FLASH_OFFSET * CurrentPage),
         sizeof(MemoryCard)); // read into variable
  SectorDirty = CheckFormatted(MemoryCard, CurrentPage);
}

void pageToggle(uint gpio, uint32_t events)
{
  gpio_acknowledge_irq(gpio, events);
  uint32_t pressTime = to_ms_since_boot(get_absolute_time());
  if ((pressTime - lastPress) >= 500)
  {
    if (CurrentPage == 8)
      CurrentPage = 1;
    else
      CurrentPage++;
    if (!PageCycle && !SectorDirty)
      PageCycle = true;
    lastPress = pressTime;
  }
}

void setPixel(uint8_t x, uint8_t y, uint16_t color)
{
  if (gpio_get(OLED_PIN))
    setPixelSSD1331(x, y, color);
  else
    setPixel1306(x + 16, y, color ? 1 : 0);
}

bool __no_inline_not_in_flash_func(vibeHandler)(struct repeating_timer *t)
{
  static uint32_t pulseTimestamp = 0;

  static uint8_t vibeCtrl = 0;
  static uint8_t vibePow = 0;
  static uint8_t vibeFreq = 0;
  static uint8_t vibeInc = 0;

  static bool commandInProgress = false;
  static bool pulseInProgress = false;
  static bool converge = false;
  static bool diverge = false;
  static bool vergenceComplete = false;

  static uint32_t vibeFreqCountLimit = 0;
  static uint32_t halfVibeFreqCountLimit = 0;
  static uint32_t vibePower = 0;

  static uint8_t convergePow = 0; // upper half of vibePow (INH)
  static uint8_t divergePow = 0;  // lower half of vibePow (EXH)

  static uint32_t inc_count = 0;

  // Check for most recent vibe command at end of each pulse
  if (!pulseInProgress)
  {
    if (purupuruUpdated)
    {
      // Update purupuru flags
      vibeCtrl = ctrl;
      vibePow = power;
      vibeFreq = freq;
      vibeInc = inc;

      // (vibeFreq + 1)/2 gives us frequency of vibe pulses. How many 500us vibeHandler cycles fit in one vibe period?
      vibeFreqCountLimit = 4000 / (vibeFreq + 1);
      halfVibeFreqCountLimit = vibeFreqCountLimit >> 1;

      convergePow = (vibePow >> 4 & 0x7);
      divergePow = (vibePow & 0x7);

      if ((divergePow == 0) && (convergePow == 0))
        vibePower = 0;
      else if (divergePow >= convergePow)
        vibePower = map_uint32(divergePow, 1, 7, 0x9FFF, 0xFFFF);
      else if (divergePow < convergePow)
        vibePower = map_uint32(convergePow, 1, 7, 0x9FFF, 0xFFFF);

      converge = (vibePow & 0x80 && convergePow) ? true : false; // is INH set and is convergePow nonzero?
      diverge = (vibePow & 0x8 && divergePow) ? true : false;    // is EXH set and is divergePow nonzero ?

      if (converge && diverge)
      { // simultaneous convergence and divegence is not supported!
        vibePower = 0;
        converge = false;
        diverge = false;
      }

      if (!converge && !diverge)
      { // if vergence is disabled, inc = 0 is treated as inc = 1
        if (inc == 0)
          inc = 1;
      }

      vibeFreqCount = 0;
      pulseInProgress = true;
      commandInProgress = true;
      purupuruUpdated = false;

      // take timestamp of pulse start for auto-stop
      AST_timestamp = to_ms_since_boot(get_absolute_time());
    }
  }

  // Pulse handling
  if (commandInProgress)
  {
    // first, enforce auto-stop (only in continuous mode)
    pulseTimestamp = to_ms_since_boot(get_absolute_time());

    // if ( (pulseTimestamp - AST_timestamp) > (AST[2] * 250) ){
    //   pwm_set_gpio_level(15, 0);
    //   vibeFreqCount = 0;
    //   pulseInProgress = false;
    //   commandInProgress = false;
    // }

    // halt all vibration instantly if vibePower == 0

    if(vibePower == 0){
      pwm_set_gpio_level(15, 0);
      pulseInProgress = false;
      commandInProgress = false;
      converge = false;
      diverge = false;
    }

    if (converge)
    {
      if (convergePow > 0)
      { // inc must be non-zero
        // First remap vibePower with decremented convergePow
        vibePower = map_uint32(convergePow, 1, 7, 0x9FFF, 0xFFFF);

        // If inc is zero, we can't converge. Bit hacky...
        if (inc == 0)
          vibeFreqCount = vibeFreqCountLimit;

        if (vibeFreqCount <= halfVibeFreqCountLimit)
        {
          pwm_set_gpio_level(15, vibePower);
          vibeFreqCount++;
        }
        else if (vibeFreqCount < vibeFreqCountLimit)
        {
          pwm_set_gpio_level(15, 0);
          vibeFreqCount++;
        }
        else
        {
          // pwm_set_gpio_level(15, 0);
          vibeFreqCount = 0;
          inc_count++;
          if (inc_count >= inc)
          {
            pulseInProgress = false;
            inc_count = 0;
            convergePow--;
          }
        }
      }
      else
      {
        commandInProgress = false;
        converge = false;
      }
    }

    else if (diverge)
    {
      if (divergePow < 8)
      { // inc must be non-zero
        // First remap non-zero vibePower with incremented divergePow
        vibePower = map_uint32(divergePow, 1, 7, 0x9FFF, 0xFFFF);

        // If inc is zero, we can't diverge. Bit hacky...
        if (inc == 0)
          vibeFreqCount = vibeFreqCountLimit;

        if (vibeFreqCount <= halfVibeFreqCountLimit)
        {
          pwm_set_gpio_level(15, vibePower);
          vibeFreqCount++;
        }
        else if (vibeFreqCount < vibeFreqCountLimit)
        {
          pwm_set_gpio_level(15, 0);
          vibeFreqCount++;
        }
        else
        {
          // pwm_set_gpio_level(15, 0);
          vibeFreqCount = 0;
          inc_count++;
          if (inc_count >= inc)
          {
            pulseInProgress = false;
            inc_count = 0;
            divergePow++;
          }
        }
      }
      else
      {
        pwm_set_gpio_level(15, 0);
        commandInProgress = false;
        diverge = false;
      }
    }

    else if (vibeFreqCount <= halfVibeFreqCountLimit)
    { // non-vergence pulses
      pwm_set_gpio_level(15, vibePower);
      vibeFreqCount++;
    }
    else if (vibeFreqCount < vibeFreqCountLimit)
    {
      pwm_set_gpio_level(15, 0);
      vibeFreqCount++;
    }
    else
    {
      // pwm_set_gpio_level(15, 0);
      vibeFreqCount = 0;
      inc_count++;
      if (inc_count >= inc)
      {
        pulseInProgress = false;
        commandInProgress = false;
        inc_count = 0;
      }
    }
  }
  return (true);
}

int main()
{
  stdio_init_all();
  // set_sys_clock_khz(175000, false); // Overclock seems to lead to instability

  multicore_launch_core1(core1_entry);

  // Controller packets
  BuildInfoPacket();
  BuildControllerPacket();

  // Subperipheral packets
  BuildACKPacket();
  BuildSubPeripheral0InfoPacket();
  BuildSubPeripheral1InfoPacket();
  BuildMemoryInfoPacket();
  BuildLCDInfoPacket();
  BuildPuruPuruInfoPacket();
  BuildDataPacket();

  SetupMapleTX();
  SetupMapleRX();

  uint StartOfPacket = 0;
  while (true)
  {
    uint EndOfPacket = multicore_fifo_pop_blocking();

    // TODO: Improve. Would be nice not to move here
    for (uint i = StartOfPacket; i < EndOfPacket; i += 4)
    {
      *(uint *)&Packet[i - StartOfPacket] = __builtin_bswap32(*(uint *)&RecieveBuffer[i & (sizeof(RecieveBuffer) - 1)]);
    }

    uint PacketSize = EndOfPacket - StartOfPacket;
    ConsumePacket(PacketSize);
    StartOfPacket = ((EndOfPacket + 3) & ~3);

    if (NextPacketSend != SEND_NOTHING)
    {
      NextPacketSend = SEND_NOTHING;
    }
  }
}