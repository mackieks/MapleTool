#include <stdint.h>
#include "maple.pio.h"
#include "pico/stdlib.h"

#define HKT7700 1 // "Seed" (standard controller)
#define HKT7300 0 // Arcade stick

#if HKT7700
#define NUM_BUTTONS 9
#elif HKT7300
#define NUM_BUTTONS 11
#endif

typedef struct PacketHeader_s {
  int8_t Command;
  uint8_t Destination;
  uint8_t Origin;
  uint8_t NumWords;
} PacketHeader;

typedef struct PacketDeviceInfo_s {
  uint Func;        // Nb. Big endian
  uint FuncData[3]; // Nb. Big endian
  int8_t AreaCode;
  uint8_t ConnectorDirection;
  char ProductName[30];
  char ProductLicense[60];
  uint16_t StandbyPower;
  uint16_t MaxPower;
} PacketDeviceInfo;

typedef struct PacketMemoryInfo_s {
  uint Func; // Nb. Big endian
  uint16_t TotalSize;
  uint16_t ParitionNumber;
  uint16_t SystemArea;
  uint16_t FATArea;
  uint16_t NumFATBlocks;
  uint16_t FileInfoArea;
  uint16_t NumInfoBlocks;
  uint8_t VolumeIcon;
  uint8_t Reserved;
  uint16_t SaveArea;
  uint16_t NumSaveBlocks;
  uint Reserved32;
} PacketMemoryInfo;

typedef struct PacketLCDInfo_s {
  uint Func;            // Nb. Big endian
  uint8_t dX;           // Number of X-axis dots
  uint8_t dY;           // Number of Y-axis dots
  uint8_t GradContrast; // Upper nybble Gradation (bits/dot), lower nybble contrast (0 to 16 steps)
  uint8_t Reserved;

} PacketLCDInfo;

typedef struct PacketTimerInfo_s {
  uint Func;            // Nb. Big endian
  uint8_t dX;           // Number of X-axis dots
  uint8_t dY;           // Number of Y-axis dots
  uint8_t GradContrast; // Upper nybble Gradation (bits/dot), lower nybble contrast (0 to 16 steps)
  uint8_t Reserved;

} PacketTimerInfo;

typedef struct PacketPuruPuruInfo_s {
  uint Func;     // Nb. Big endian
  uint8_t VSet0; // Upper nybble is number of vibration sources, lower nybble is vibration source location and vibration source axis
  uint8_t Vset1; // b7: Variable vibration intensity flag, b6: Continuous vibration flag, b5: Vibration direction control flag, b4: Arbitrary waveform flag
                 // Lower nybble is Vibration Attribute flag (fixed freq. or ranged freq.)
  uint8_t FMin;  // Minimum vibration frequency when VA = 0000 (Ffix when VA = 0001, reserved when VA = 1111)
  uint8_t FMax;  // Maximum vibration frequency when VA = 0000 (reserved when VA= 0001 or 1111)

} PacketPuruPuruInfo;

typedef struct PacketControllerCondition_s {
  uint Condition; // Nb. Big endian
  uint16_t Buttons;
  uint8_t RightTrigger;
  uint8_t LeftTrigger;
  uint8_t JoyX;
  uint8_t JoyY;
  uint8_t JoyX2;
  uint8_t JoyY2;
} PacketControllerCondition;

typedef struct PacketPuruPuruCondition_s {
  uint Func;       // Nb. Big endian
  uint8_t VSource; // Vibration control
  uint8_t Power;   // Vibration intensity
  uint8_t Freq;    // Vibration frequency
  uint8_t Inc;     // Vibration inclination
} PacketPuruPuruCondition;

typedef struct PacketBlockRead_s {
  uint Func; // Nb. Big endian
  uint Address;
  uint8_t Data[512];
} PacketBlockRead;

typedef struct FPuruPuruBlockReadPacket_s {
  uint Func; // Nb. Big endian
  uint Address;
  uint8_t Data[4];
} FPuruPuruBlockReadPacket;

typedef struct FACKPacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  uint CRC;
} FACKPacket;

typedef struct FInfoPacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  PacketDeviceInfo Info;
  uint CRC;
} FInfoPacket;

typedef struct FMemoryInfoPacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  PacketMemoryInfo Info;
  uint CRC;
} FMemoryInfoPacket;

typedef struct FLCDInfoPacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  PacketLCDInfo Info;
  uint CRC;
} FLCDInfoPacket;

typedef struct FTimerInfoPacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  PacketLCDInfo Info;
  uint CRC;
} FTimerInfoPacket;

typedef struct FPuruPuruInfoPacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  PacketPuruPuruInfo Info;
  uint CRC;
} FPuruPuruInfoPacket;

typedef struct FPuruPuruConditionPacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  PacketPuruPuruCondition Condition;
  uint CRC;
} FPuruPuruConditionPacket;

typedef struct FControllerPacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  PacketControllerCondition Controller;
  uint CRC;
} FControllerPacket;

typedef struct FPuruPuruBlockReadResponsePacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  FPuruPuruBlockReadPacket PuruPuruBlockRead;
  uint CRC;
} FPuruPuruBlockReadResponsePacket;

typedef struct FBlockReadResponsePacket_s {
  uint BitPairsMinus1;
  PacketHeader Header;
  PacketBlockRead BlockRead;
  uint CRC;
} FBlockReadResponsePacket;

typedef struct ButtonInfo_s {
  int InputIO;
  int DCButtonMask;
} ButtonInfo;

static ButtonInfo ButtonInfos[NUM_BUTTONS] = {
    {0, 0x0004}, // A
    {1, 0x0002}, // B
    {4, 0x0400}, // X
    {5, 0x0200}, // Y
    {6, 0x0010}, //  Up
    {7, 0x0020}, // Down
    {8, 0x0040}, // Left
    {9, 0x0080}, // Right
    {10, 0x0008} // Start
#if HKT7300
    ,
    {16, 0x0001}, // C
    {17, 0x0100}  // Z
#endif
};