#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "format.h"
#ifdef PICO_HW
#include "hardware/flash.h"
#endif

enum EFileType
{
	FileType_NoFile = 0x00,
	FileType_Data = 0x33,
	FileType_Game = 0xCC
};

enum EFATType
{
	FATType_Free = 0xFFFC,
	FATType_EOF = 0xFFFA
};

typedef struct Date_s
{
	uint8_t Century;
	uint8_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t DayOfWeek;
} Date;

typedef struct DirectoryEntry_s
{
	uint8_t FileType;
	uint8_t CopyProtect;
	uint16_t FirstBlock;
	char Name[12];
	Date Creation;
	uint16_t SizeInBlocks;
	uint16_t HeaderOffset;
	uint8_t Padding[4];
} DirectoryEntry;

typedef struct RootBlock_s
{
	uint8_t Magic[16];
	uint8_t CustomColor;
	uint8_t CustomColorBlue;
	uint8_t CustomColorGreen;
	uint8_t CustomColorRed;
	uint8_t CustomColorAlpha;
	uint8_t Padding[27];
	Date Format;
	uint8_t Padding2[8];
    uint16_t TotalSize;             // Seems to be the same as memory info but with a set bit in last uint
	uint16_t ParitionNumber;
	uint16_t SystemArea;
	uint16_t FATBlock;
	uint16_t FATSizeInBlocks;
	uint16_t DirectoryBlock;
	uint16_t DirectorySizeInBlocks;
	uint16_t IconShape;
	uint16_t NumberOfUserBlocks;
    uint16_t NumSaveBlocks;
    uint32_t Unknown;
} RootBlock;

// ICONDATA_VMS generated by make_icon.cpp
static const uint8_t IconData[0x2C0] =
/* G:\ICONDATA (1).VMS (8/21/2021 11:28:53 AM)
   StartOffset(h): 00000000, EndOffset(h): 000002CF, Length(h): 000002D0 */
{
	0x2E, 0x2F, 0x75, 0x70, 0x6C, 0x6F, 0x61, 0x64, 0x2F, 0x65, 0x62, 0x37,
	0x61, 0x61, 0x30, 0x36, 0x20, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x7E, 0x7F, 0x00, 0x00, 0x7E, 0x7F, 0x00, 0x01, 0xFF, 0xFF, 0xC0,
	0x02, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00, 0x20,
	0x02, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00, 0x20, 0x02, 0x7F, 0xFF, 0x20,
	0x02, 0x7F, 0xFF, 0x20, 0x02, 0x7F, 0xFF, 0x20, 0x02, 0x7F, 0xFF, 0x20,
	0x02, 0x7F, 0xFF, 0x20, 0x02, 0x7F, 0xFF, 0x20, 0x02, 0x7F, 0xFF, 0x20,
	0x02, 0x7F, 0xFF, 0x20, 0x02, 0x7F, 0xFF, 0x20, 0x02, 0x7F, 0xFF, 0x20,
	0x02, 0x7F, 0xFF, 0x20, 0x02, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00, 0x20,
	0x02, 0x00, 0xD8, 0x20, 0x02, 0x38, 0xDB, 0xA0, 0x02, 0x7C, 0x03, 0xA0,
	0x02, 0x7C, 0x73, 0xA0, 0x02, 0x7C, 0x70, 0x20, 0x02, 0x38, 0x70, 0x20,
	0x02, 0x00, 0x00, 0x20, 0x01, 0x00, 0x00, 0x40, 0x00, 0xF0, 0x07, 0x80,
	0x00, 0x0F, 0xF8, 0x00, 0xFF, 0xFF, 0xBB, 0xFB, 0x00, 0xF0, 0x80, 0xF0,
	0x88, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x22, 0x22, 0x20,
	0x02, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x02, 0x22, 0x22, 0x20, 0x02, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x02, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
	0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x11, 0x11, 0x11, 0x11,
	0x11, 0x11, 0x11, 0x11, 0x11, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21,
	0x11, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x20, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x10, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x10, 0x10, 0x20, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20, 0x14, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x24,
	0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x12, 0x33, 0x33, 0x33,
	0x33, 0x33, 0x33, 0x32, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x12, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x32, 0x10, 0x20, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20, 0x12, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x32,
	0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x12, 0x33, 0x33, 0x33,
	0x33, 0x33, 0x33, 0x32, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x12, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x32, 0x10, 0x20, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20, 0x12, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x32,
	0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x12, 0x33, 0x33, 0x33,
	0x33, 0x33, 0x33, 0x32, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x12, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x32, 0x10, 0x20, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20, 0x12, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x32,
	0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x14, 0x22, 0x22, 0x22,
	0x22, 0x22, 0x22, 0x24, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x10, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x10, 0x10, 0x20, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x10, 0x11, 0x10, 0x00,
	0x44, 0x04, 0x40, 0x00, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x11, 0x22, 0x21, 0x00, 0x44, 0x04, 0x41, 0x42, 0x40, 0x20, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20, 0x12, 0x22, 0x22, 0x10, 0x00, 0x00, 0x01, 0x22,
	0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x12, 0x22, 0x22, 0x10,
	0x04, 0x24, 0x11, 0x42, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x12, 0x22, 0x22, 0x10, 0x02, 0x22, 0x10, 0x00, 0x10, 0x20, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20, 0x11, 0x22, 0x21, 0x00, 0x04, 0x24, 0x10, 0x00,
	0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x12, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x11, 0x11, 0x11, 0x11, 0x12, 0x22,
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22,
	0x22, 0x22, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x1A, 0x1A, 0x1A,
	0x1A, 0x1A, 0x1A, 0x1A, 0x1A, 0x1A, 0x1A, 0x1A, 0x1A, 0x1A, 0x1A, 0x1A
};


static void AllocateFAT(uint16_t *FAT, uint16_t StartBlock, uint16_t NumBlocks)
{
	uint32_t EndBlock = StartBlock + NumBlocks - 1;
	FAT[StartBlock] = FATType_EOF;
	while (StartBlock != EndBlock)
	{
		StartBlock++;
		FAT[StartBlock]= StartBlock - 1;
	}
}

uint32_t CheckFormatted(uint8_t* MemoryCard)
{
    uint32_t SectorDirty = 0;
    const RootBlock Root =
	{
		{0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55},
		1, 0, 255, 255, 255, {0}, {0x20, 0x21, 0x03, 0x02, 0x09, 0x00, 0x00, 0x01}, {0},
		CARD_BLOCKS - 1, 0, ROOT_BLOCK, FAT_BLOCK, NUM_FAT_BLOCKS, DIRECTORY_BLOCK, NUM_DIRECTORY_BLOCKS, 0, SAVE_BLOCK, NUM_SAVE_BLOCKS, 0x800000
	};
    if (memcmp(&MemoryCard[ROOT_BLOCK * BLOCK_SIZE], Root.Magic, sizeof(Root.Magic)) != 0)
    {
		// If not formatted then initialize ourselves. Saves user a step + means we can have a fancy icon
		uint32_t StartOfDirectoryBlock = Root.DirectoryBlock - Root.DirectorySizeInBlocks + 1;
		memset(&MemoryCard[StartOfDirectoryBlock * BLOCK_SIZE], 0, (256 - StartOfDirectoryBlock) * BLOCK_SIZE);
		memcpy(&MemoryCard[ROOT_BLOCK * BLOCK_SIZE], &Root, sizeof(Root));

		const DirectoryEntry IconDataVMS = {FileType_Data, 0, SAVE_BLOCK - 2, "ICONDATA_VMS", {0x20, 0x21, 0x03, 0x02, 0x09, 0x00, 0x00, 0x01}, 2, 0};
		memcpy(&MemoryCard[Root.DirectoryBlock * BLOCK_SIZE], &IconDataVMS, sizeof(IconDataVMS));
		memcpy(&MemoryCard[IconDataVMS.FirstBlock * BLOCK_SIZE], &IconData, sizeof(IconData));

		uint32_t StartOfFATBlock = Root.FATBlock - Root.FATSizeInBlocks + 1;
		uint16_t *FAT = (uint16_t*)&MemoryCard[StartOfFATBlock * BLOCK_SIZE];
		for (uint32_t Block = 0; Block < Root.FATSizeInBlocks * (BLOCK_SIZE / sizeof(uint16_t)); Block++)
		{
			FAT[Block] = FATType_Free;
		}
		AllocateFAT(FAT, ROOT_BLOCK, 1);
		AllocateFAT(FAT, StartOfFATBlock, Root.FATSizeInBlocks);
		AllocateFAT(FAT, StartOfDirectoryBlock, Root.DirectorySizeInBlocks);
		FAT[IconDataVMS.FirstBlock] = IconDataVMS.FirstBlock + 1;
		FAT[IconDataVMS.FirstBlock + 1] = FATType_EOF;

#if PICO_HW
		// Everything above StartOfDirectoryBlock now needs writing to flash + icon data
		SectorDirty |= ~((1u << ((StartOfDirectoryBlock * BLOCK_SIZE) / FLASH_SECTOR_SIZE)) - 1);
		SectorDirty |= 1u << ((IconDataVMS.FirstBlock * BLOCK_SIZE) / FLASH_SECTOR_SIZE);
 #endif
	}
    return SectorDirty;
}