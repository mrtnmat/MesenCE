#pragma once
#include "NES/Mappers/Nintendo/MMC1.h"

//Mapper 155 = MMC1A: PRG-RAM always enabled, $E000 bit 4 = PRG A17 bypass
class MMC1_155 : public MMC1
{
protected:
	void InitMapper() override
	{
		//[lab fork] model the MMC1A (see MMC1::_isMmc1a) instead of dropping $E000 bit 4
		_isMmc1a = true;
		MMC1::InitMapper();
	}
};
