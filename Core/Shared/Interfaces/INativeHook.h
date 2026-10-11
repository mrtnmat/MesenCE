#pragma once
#include "pch.h"

enum class MemoryOperationType;

//[lab fork] In-process client hook (used by the headless MesenServer).
//OnFrame runs on the emulation thread, once before the first frame and then after every frame,
//at an instruction boundary: the hook may read/write memory and save/load states from there.
//Memory callbacks are opt-in through the Emulator::HookCpuRead/HookCpuWrite/HookPpuRead flags.
class INativeHook
{
public:
	virtual ~INativeHook() {}

	//Return false to stop emulation.
	virtual bool OnFrame(bool beforeFirstFrame) = 0;

	virtual void OnCpuRead(uint32_t addr, uint8_t value, MemoryOperationType opType) {}
	virtual void OnCpuWrite(uint32_t addr, uint8_t value, MemoryOperationType opType) {}
	virtual void OnPpuRead(uint32_t addr, uint8_t value, MemoryOperationType opType) {}
};
