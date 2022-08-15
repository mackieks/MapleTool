/**
 * MapleTool
 * Forked from Pop'n Music Controller
 * Dreamcast Maple Bus Transiever example for Raspberry Pi Pico (RP2040)
 * (C) Charlie Cole 2021
 *
 * Changes by Mackie Kannard-Smith 2022
 *
 * Dreamcast controller connector pin 1 (Maple Bus A) to GP11 (MAPLE_A)
 * Dreamcast controller connector pin 5 (Maple Bus B) to GP12 (MAPLE_B)
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

#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "state_machine.h"
#include "hardware/vreg.h"


// Maple Bus Defines and Funcs

#define SHOULD_SEND 0  // Set to zero to sniff two devices sending signals to each other
#define SHOULD_PRINT 0 // Nice for debugging but can cause timing issues

#define MAPLE_A 11
#define MAPLE_B 12
#define PICO_PIN1_PIN_RX MAPLE_A
#define PICO_PIN5_PIN_RX MAPLE_B

#define ADDRESS_DREAMCAST 0
#define ADDRESS_CONTROLLER 0x20
#define ADDRESS_SUBPERIPHERAL0 0x01
#define ADDRESS_SUBPERIPHERAL1 0x02

#define ADDRESS_PORT_MASK 0xC0
#define ADDRESS_PERIPHERAL_MASK (~ADDRESS_PORT_MASK)

#define TXPIO pio0
#define RXPIO pio1

#define SWAP4(x)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       \
  do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 \
    x = __builtin_bswap32(x);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          \
  } while (0)

typedef enum ESendState_e {
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

enum ECommands {
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

enum EFunction {
  FUNC_CONTROLLER = 1,  // FT0
  FUNC_MEMORY_CARD = 2, // FT1
  FUNC_LCD = 4,         // FT2
  FUNC_TIMER = 8,       // FT3
  FUNC_VIBRATION = 256  // FT8
};

// Buffers
static uint8_t RecieveBuffer[4096] __attribute__((aligned(4))); // Ring buffer + strlen(buffer) for reading packets
static uint8_t Packet[1024 + 8] __attribute__((aligned(4)));    // Temp buffer + strlen(buffer) for consuming packets (could remove)

static ESendState NextPacketSend = SEND_NOTHING;
static uint OriginalControllerCRC = 0;
static uint OriginalReadBlockResponseCRC = 0;
static uint TXDMAChannel = 0;

static char buffer[512] = {0};

bool __not_in_flash_func(ConsumePacket)(uint Size) {
  if ((Size & 3) == 1) // Even number of words + CRC
  {
    Size--; // Drop CRC byte
    if (Size > 0) {
      PacketHeader *Header = (PacketHeader *)Packet;
      uint *PacketData = (uint *)(Header + 1);
      if (Size == (Header->NumWords + 1) * 4) {
        // Mask off port number
        Header->Destination &= ADDRESS_PERIPHERAL_MASK;
        Header->Origin &= ADDRESS_PERIPHERAL_MASK;

        // TO-DO: Print packet info (check Pop'n Music source)

        // case statement for function code makes sense here
        // should format packet info prettily

        // Use function code and if-else to determine how to format "CMD_RESPOND" data
        // for instance if Func = controller, CMD_RESPOND_DATA_TRANSFER is a controller condition packet

        switch (Header->Command)
        {
          case CMD_REQUEST_DEVICE_INFO:
          {
            PacketDeviceInfo *DeviceInfo = (PacketDeviceInfo *)(Header + 1);

            SWAP4(DeviceInfo->Func);
            SWAP4(DeviceInfo->FuncData[0]);
            SWAP4(DeviceInfo->FuncData[1]);
            SWAP4(DeviceInfo->FuncData[2]);

            sprintf(buffer + strlen(buffer),"DEVICE_INFO\n");
            // sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x\n", Header->Command, Header->Destination,
            //       Header->Origin, Header->NumWords);
                  
            return(true);
          }
          case CMD_REQUEST_EXTENDED_DEVICE_INFO:
          {
            PacketDeviceInfo *DeviceInfo = (PacketDeviceInfo *)(Header + 1);

            SWAP4(DeviceInfo->Func);
            SWAP4(DeviceInfo->FuncData[0]);
            SWAP4(DeviceInfo->FuncData[1]);
            SWAP4(DeviceInfo->FuncData[2]);

            sprintf(buffer + strlen(buffer),"ALL_DEVICE_INFO\n");
            // sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x\n", Header->Command, Header->Destination,
            //       Header->Origin, Header->NumWords);
                  
            return(true);
          }
          case CMD_RESET_DEVICE:
          {
            sprintf(buffer + strlen(buffer),"DEVICE_RESET\n");
            // sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x\n", Header->Command, Header->Destination,
            //       Header->Origin, Header->NumWords);
            
            return(true);
          }
          case CMD_SHUTDOWN_DEVICE:
          {
            sprintf(buffer + strlen(buffer),"DEVICE_SHUTDOWN\n");
            // sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x\n", Header->Command, Header->Destination,
            //       Header->Origin, Header->NumWords);
            
            return(true);
          }
          case CMD_RESPOND_DEVICE_INFO:
          {
            PacketDeviceInfo *DeviceInfo = (PacketDeviceInfo *)(Header + 1);

            SWAP4(DeviceInfo->Func);
            SWAP4(DeviceInfo->FuncData[0]);
            SWAP4(DeviceInfo->FuncData[1]);
            SWAP4(DeviceInfo->FuncData[2]);

            sprintf(buffer + strlen(buffer),"DEVICE_INFO\n");
            sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x\n", Header->Command, Header->Destination,
                  Header->Origin, Header->NumWords);

            // sprintf(buffer + strlen(buffer),"Info:\nFunction Type: %08x (%08x %08x %08x) Area: %d Dir: %d Name: %.*s License: %.*s Pwr: %d->%d\n",
            //       DeviceInfo->Func, DeviceInfo->FuncData[0], DeviceInfo->FuncData[1], DeviceInfo->FuncData[2],
            //       DeviceInfo->AreaCode, DeviceInfo->ConnectorDirection, 30, DeviceInfo->ProductName, 60, DeviceInfo->ProductLicense,
            //       DeviceInfo->StandbyPower, DeviceInfo->MaxPower);
                  
            return(true);
          }
          case CMD_RESPOND_EXTENDED_DEVICE_INFO:
          {
            PacketDeviceInfo *DeviceInfo = (PacketDeviceInfo *)(Header + 1);

            SWAP4(DeviceInfo->Func);
            SWAP4(DeviceInfo->FuncData[0]);
            SWAP4(DeviceInfo->FuncData[1]);
            SWAP4(DeviceInfo->FuncData[2]);

            sprintf(buffer + strlen(buffer),"ALL_DEVICE_INFO\n");
            sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x\n", Header->Command, Header->Destination,
                  Header->Origin, Header->NumWords);

            // sprintf(buffer + strlen(buffer),"Info:\nFunction Type: %08x (%08x %08x %08x) Area: %d Dir: %d Name: %.*s License: %.*s Pwr: %d->%d FreeDevStatus: %.*s\n",
            //       DeviceInfo->Func, DeviceInfo->FuncData[0], DeviceInfo->FuncData[1], DeviceInfo->FuncData[2],
            //       DeviceInfo->AreaCode, DeviceInfo->ConnectorDirection, 30, DeviceInfo->ProductName, 60, DeviceInfo->ProductLicense,
            //       DeviceInfo->StandbyPower, DeviceInfo->MaxPower, DeviceInfo->FreeDeviceStatus, 80);
                  
            return(true);
          }
          case CMD_RESPOND_COMMAND_ACK:
          {
            sprintf(buffer + strlen(buffer),"DEVICE_REPLY_ACK\n");
            // sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x\n", Header->Command, Header->Destination,
            //       Header->Origin, Header->NumWords);
            
            return(true);
          }
          case CMD_RESPOND_DATA_TRANSFER:
          {
            sprintf(buffer + strlen(buffer),"DATA_TRANSFER\n");
            sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x\n", Header->Command, Header->Destination,
                  Header->Origin, Header->NumWords);

            uint func = *(uint *)(Header + 1);

            // if(func == __bswap32(FUNC_CONTROLLER)){ // Reformat to include func + all other fields
            //   PacketControllerCondition *ControllerCondition = (PacketControllerCondition *)(Header + 1);
            //   SWAP4(ControllerCondition->Condition);

            //   sprintf(buffer + strlen(buffer),"Controller Condition:\nFunction Type: %08x Buttons: %02x LT:%d RT:%d Joy: %d %d Joy2: %d %d\n",
						// 			   ControllerCondition->Condition, ControllerCondition->Buttons, ControllerCondition->LeftTrigger, ControllerCondition->RightTrigger,
						// 			   ControllerCondition->JoyX, ControllerCondition->JoyY,
						// 			   ControllerCondition->JoyX2, ControllerCondition->JoyY2);
            //   return(true);
            // }
            // else if(func == __bswap32(FUNC_MEMORY_CARD)){
            //   // Could be responding to GET_MEDIA_INFO or BLOCK_READ
            //   // GET_MEDIA_INFO response always has Data Size = 0x07
            //   if(Header->NumWords == 0x07){ // GET_MEDIA_INFO response
            //     PacketMemoryInfo *MemoryInfo = (PacketMemoryInfo *)(Header + 1);
            //     SWAP4(MemoryInfo->Func);

            //     sprintf(buffer + strlen(buffer),"Memory Info:\nFunction Type: %08x TotalSize: %04x PartitionNumber: %04x System Area: %04x FAT Area: %04x\n \
            //     NumFATBlocks: %04x FileInfoArea: %04x NumInfoBlocks: %04x Volume Icon: %02x Reserved:%02x\n \
            //     SaveArea: %04x NumSaveBlocks: %04x Reserved: %08x\n", MemoryInfo->Func, MemoryInfo->TotalSize, MemoryInfo->PartitionNumber,
            //     MemoryInfo->SystemArea, MemoryInfo->FATArea, MemoryInfo->NumFATBlocks, MemoryInfo->NumFATBlocks, MemoryInfo->FileInfoArea,
            //     MemoryInfo->NumInfoBlocks, MemoryInfo->VolumeIcon, MemoryInfo->Reserved, MemoryInfo->SaveArea, MemoryInfo->NumSaveBlocks,
            //     MemoryInfo->Reserved32);

            //     return(true);
            //   }
            //   else { // BLOCK_READ response
            //     PacketBlockRead *BlockRead = (PacketBlockRead *)(Header + 1);
            //     SWAP4(BlockRead->Func);

            //     sprintf(buffer + strlen(buffer),"Block Read:\nFunction Type: %08x PartitionNumber (PT): %02x Phase: %02x Block No: %04x\nBlock Data: NOT PRINTED",
            //             BlockRead->Func, BlockRead->Address >> 24, (BlockRead->Address & 0xff0000) >> 16, BlockRead->Address & 0xffff);

            //     return(true);
            //   }
            // }
            return(true);
          }
          case CMD_GET_CONDITION:
          {
            uint func = *(uint *)(Header + 1);

            sprintf(buffer + strlen(buffer),"GET_CONDITION\n");
            sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x Function Type:%08x\n", Header->Command, Header->Destination,
                  Header->Origin, Header->NumWords, __bswap32(func));
            
            return(true);
          }
          case CMD_GET_MEDIA_INFO:
          {
            uint func = *(uint *)(Header + 1);
            uint data = *(uint *)(Header + 2);

            sprintf(buffer + strlen(buffer),"GET_MEDIA_INFO\n");
            sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x Function Type: %08x Partition Number: %02x Reserved: %06x\n", Header->Command, Header->Destination,
                  Header->Origin, Header->NumWords, __bswap32(func), __bswap32(data) >> 24, __bswap32(data) & 0xffffff);
            
            return(true);
          }
          case CMD_BLOCK_READ:
          {
            uint func = *(uint *)(Header + 1);
            uint data = *(uint *)(Header + 2);

            sprintf(buffer + strlen(buffer),"BLOCK_READ\n");
            sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x Function Type: %08x Partition Number: %02x Phase: %02x Block No: %04x\n", Header->Command, Header->Destination,
                 Header->Origin, Header->NumWords, __bswap32(func), __bswap32(data) >> 24, (__bswap32(data) & 0xff0000) >> 16, __bswap32(data) & 0xffff);
            
            return(true);
          }
          case CMD_BLOCK_WRITE:
          {
            uint func = *(uint *)(Header + 1);
            uint data = *(uint *)(Header + 2);

            sprintf(buffer + strlen(buffer),"BLOCK_WRITE\n");
            sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x Function Type: %08x Partition Number: %02x Phase: %02x Block No: %04x Block Data: NOT PRINTED\n", Header->Command, Header->Destination,
                  Header->Origin, Header->NumWords, __bswap32(func), __bswap32(data) >> 24, (__bswap32(data) & 0xff0000) >> 16, __bswap32(data) & 0xffff);
            
            return(true);
          }
          case CMD_BLOCK_COMPLETE_WRITE:
          {
            uint func = *(uint *)(Header + 1);
            uint data = *(uint *)(Header + 2);

            sprintf(buffer + strlen(buffer),"GET_LAST_ERROR (COMPLETE_BLOCK_WRITE)\n");
            // sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x Function Type: %08x Partition Number: %02x Phase: %02x Block No: %04x Block Data: NOT PRINTED\n", Header->Command, Header->Destination,
            //       Header->Origin, Header->NumWords, __bswap32(func), __bswap32(data) >> 24, (__bswap32(data) & 0xff0000) >> 16, __bswap32(data) & 0xffff);
            
            return(true);
          }
          case CMD_SET_CONDITION:
          {
            uint func = *(uint *)(Header + 1);
            uint data = *(uint *)(Header + 2);

            sprintf(buffer + strlen(buffer),"SET_CONDITION\n");
            // sprintf(buffer + strlen(buffer),"Command: %04x Destination: %04x Origin: %04x Data Length: %04x Function Type: %08x Partition Number: %02x Phase: %02x Block No: %04x Block Data: NOT PRINTED\n", Header->Command, Header->Destination,
            //       Header->Origin, Header->NumWords, __bswap32(func), __bswap32(data) >> 24, (__bswap32(data) & 0xff0000) >> 16, __bswap32(data) & 0xffff);
            
            return(true);
          }
          case CMD_RESPOND_FILE_ERROR:
          {
            sprintf(buffer + strlen(buffer),"CMD_RESPOND_FILE_ERROR\n");
            return(true);
          }
          case CMD_RESPOND_SEND_AGAIN:
          {
            sprintf(buffer + strlen(buffer),"CMD_RESPOND_SEND_AGAIN\n");
            return(true);
          }
          case CMD_RESPOND_FUNC_CODE_UNSUPPORTED:
          {
            sprintf(buffer + strlen(buffer),"CMD_RESPOND_FUNC_CODE_UNSUPPORTED\n");
            return(true);
          }
          case CMD_NO_RESPONSE:
          {
            sprintf(buffer + strlen(buffer),"CMD_NO_RESPONSE\n");
            return(true);
          }
          default:
          {
            sprintf(buffer + strlen(buffer),"unknown command!\n");
            return(true);
          }
        }     
        return true;
      }
    }
  }
  return false;
}

// *IMPORTANT* This function must be in RAM. Will be too slow if have to fetch
// code from flash
static void __not_in_flash_func(core1_entry)(void) {
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
  while ((RXPIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB))) == 0) {
    pio_sm_get(RXPIO, 0);
  }

  while (true) {
    // Worst case we could have only 0.5us (~65 cycles) to process each byte if
    // want to keep up real time In practice we have around 4us on average so
    // this code is easily fast enough
    while ((RXPIO->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB))) != 0)
      ;
    const uint8_t Value = RXPIO->rxf[0];
    StateMachine M = Machine[State][Value];
    State = M.NewState;
    if (M.Reset) {
      Offset = StartOfPacket;
      Byte = 0;
      XOR = 0;
    }
    Byte |= SetBits[M.SetBitsIndex][0];
    if (M.Push) {
      RecieveBuffer[Offset & (sizeof(RecieveBuffer) - 1)] = Byte;
      XOR ^= Byte;
      Byte = SetBits[M.SetBitsIndex][1];
      Offset++;
    }
    if (M.End) {
      if (XOR == 0) {
        if (multicore_fifo_wready()) {
          // multicore_fifo_push_blocking(Offset);  //Don't call as needs all be in RAM. Inlined below
          sio_hw->fifo_wr = Offset;
          __sev();
          StartOfPacket = ((Offset + 3) & ~3); // Align up for easier swizzling
        } else {
          //#if !SHOULD_PRINT // Core can be too slow due to printing
          sprintf(buffer + strlen(buffer),"Missed a packet.\n");
          //#endif
        }
      }
    }
    if ((RXPIO->fstat & (1u << (PIO_FSTAT_RXFULL_LSB))) != 0) {
      // Should be a panic but the inlining of multicore_fifo_push_blocking caused it to fire
      // Weirdly after changing this to a printf it never gets called :/
      // Something to work out...
      sprintf(buffer + strlen(buffer),"Probably overflowed. This code isn't fast enough :(\n");
    }
  }
}

void SetupMapleTX() {
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

void __not_in_flash_func(SetupMapleRX)() {
  uint RXPIOOffsets[3] = {pio_add_program(RXPIO, &maple_rx_triple1_program), pio_add_program(RXPIO, &maple_rx_triple2_program), pio_add_program(RXPIO, &maple_rx_triple3_program)};
  maple_rx_triple_program_init(RXPIO, RXPIOOffsets, PICO_PIN1_PIN_RX, PICO_PIN5_PIN_RX, 3.0f);

  // Make sure core1 is ready to say we are ready
  multicore_fifo_pop_blocking();
  multicore_fifo_push_blocking(0);

  pio_sm_set_enabled(RXPIO, 1, true);
  pio_sm_set_enabled(RXPIO, 2, true);
  pio_sm_set_enabled(RXPIO, 0, true);
}

int __not_in_flash_func(main)() {
  stdio_init_all();
  vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
  set_sys_clock_khz(150000, true);

  multicore_launch_core1(core1_entry);

  SetupMapleTX();
  SetupMapleRX();

  uint StartOfPacket = 0;
  while (true) {
    uint EndOfPacket = multicore_fifo_pop_blocking();

    // TODO: Improve. Would be nice not to move here
    for (uint i = StartOfPacket; i < EndOfPacket; i += 4) {
      *(uint *)&Packet[i - StartOfPacket] = __builtin_bswap32(*(uint *)&RecieveBuffer[i & (sizeof(RecieveBuffer) - 1)]);
    }

    uint PacketSize = EndOfPacket - StartOfPacket;
    ConsumePacket(PacketSize);
    StartOfPacket = ((EndOfPacket + 3) & ~3);

    printf("%s", buffer);
    memset(buffer, 0, sizeof(buffer + strlen(buffer))/sizeof(char));

    // if (NextPacketSend != SEND_NOTHING) {
    //   NextPacketSend = SEND_NOTHING;
    // }
  }
}