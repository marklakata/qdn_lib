#ifndef _G8_UPGRADE_H_
#define _G8_UPGRADE_H_

#include <stdint.h>

_EXTERN_C
typedef struct {
	uint32_t    CurrentAppBank;
	uint32_t    AppBoot0;
	uint32_t    AppBoot1;
	uint32_t    AppGoldenCRC;
	uint32_t    AppCalcedCRC;

    uint32_t    SpareBoot0;
    uint32_t    SpareBoot1;
	uint32_t    SpareGoldenCRC;
	uint32_t    SpareCalcedCRC;

	uint32_t    SpareMemStart;
	uint32_t    SpareMemEnd;
} UpgradeRegisters_t;

extern UpgradeRegisters_t upgradeRegisters;

void     G8_UpgradeInit(void);
int16_t  G8_UpgradeErase(uint8_t* cmdData, uint16_t sizeofCmdData, uint16_t start, uint16_t end);
int16_t  G8_UpgradeWriteData(const uint8_t* ptr, uint16_t sizeofCmdData, uint32_t length);
int16_t  G8_UpgradeSwitch(void); // switches to the spare page
int16_t  G8_UpgradeCommit(void); // commits the current page as the new default page

//int16_t  G8_UpgradeReadRegister(uint8_t* cmdData, uint16_t sizeOfCmdData, uint16_t start, uint16_t end);

int16_t  G8_UpgradeEraseAll(void);
int16_t  G8_UpgradeHexLine(char* hexline);
void     G8_CompleteUpgrade(void);
_END_EXTERN_C

#ifndef G8_UPGRADE_ERRNO__BASE
#define G8_UPGRADE_ERRNO__BASE -1000
#endif

#define G8_UPGRADE_ERRNO_SUCCESS                    (0)
#define G8_UPGRADE_ERRNO_RANGE_ERR                  (G8_UPGRADE_ERRNO__BASE - 0) //
#define G8_UPGRADE_ERRNO_FLASH_ADDRESS_OUT_OF_SPARE (G8_UPGRADE_ERRNO__BASE - 1)
#define G8_UPGRADE_ERRNO_BAD_PARAM                  (G8_UPGRADE_ERRNO__BASE - 2) // arguments to erase or write are corruptetd
#define G8_UPGRADE_ERRNO_FLASH_BAD_START_ADDRESS    (G8_UPGRADE_ERRNO__BASE - 3)
#define G8_UPGRADE_ERRNO_FLASH_BAD_LENGTH           (G8_UPGRADE_ERRNO__BASE - 4)
#define G8_UPGRADE_ERRNO_FLASH_BUFFER_ODD_SIZE      (G8_UPGRADE_ERRNO__BASE - 5)
#define G8_UPGRADE_ERRNO_BAD_FLASH_CRC              (G8_UPGRADE_ERRNO__BASE - 6)
#define G8_UPGRADE_ERRNO_BAD_INTEL_HEX_LINE         (G8_UPGRADE_ERRNO__BASE - 7)
#define G8_UPGRADE_ERRNO_BAD_CHECKSUM               (G8_UPGRADE_ERRNO__BASE - 8)


// This function is NOT defined in g8_upgrade.h. It must be defined in the user's code.
// It must convert 'len' bytes of *outgoing to 'len' bytes of *incoming, and return 0 on success.
_EXTERN_C
int16_t FirmwareDecode(uint8_t* outgoing, const uint8_t* incoming, int len);
_END_EXTERN_C

#endif
