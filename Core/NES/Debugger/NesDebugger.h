#pragma once
#include "pch.h"
#include "Debugger/DebugTypes.h"
#include "Debugger/IDebugger.h"

class NesConsole;
class Disassembler;
class Debugger;
class CallstackManager;
class MemoryAccessCounter;
class CodeDataLogger;
class EmuSettings;
class ScriptManager;
class BreakpointManager;
class IAssembler;
class BaseEventManager;
class NesTraceLogger;
class NesPpuTools;

class Emulator;
class NesCpu;
class DummyNesCpu;
class BaseNesPpu;
class BaseMapper;
class NesMemoryManager;
class ISerializable;

enum class MemoryOperationType;

class NesDebugger final : public IDebugger
{
	Debugger* _debugger;
	Emulator* _emu;
	Disassembler* _disassembler;
	MemoryAccessCounter* _memoryAccessCounter;
	EmuSettings* _settings;

	NesConsole* _console;
	NesCpu* _cpu;
	BaseMapper* _mapper;
	NesMemoryManager* _memoryManager;

	unique_ptr<CodeDataLogger> _codeDataLogger;
	unique_ptr<CodeDataLogger> _chrRomCdl;
	unique_ptr<BaseEventManager> _eventManager;
	unique_ptr<IAssembler> _assembler;
	unique_ptr<CallstackManager> _callstackManager;
	unique_ptr<BreakpointManager> _breakpointManager;
	unique_ptr<NesTraceLogger> _traceLogger;
	unique_ptr<NesPpuTools> _ppuTools;

	uint8_t _prevOpCode = 0xFF;

	//[lab fork] banked CDL (#9): per PRG-ROM byte, first CPU address it was executed/read at and a mask of
	//8KB CPU windows (bit n = $n000*2) it was seen in; call edges (#10) as absolute addresses
	vector<uint16_t> _cdlCpuAddr;
	vector<uint8_t> _cdlWindows;
	vector<uint8_t> _edges;
	std::unordered_set<uint64_t> _edgeKeys;
	__forceinline void LogPrgAccess(int32_t prgAddr, uint16_t cpuAddr)
	{
		if(_cdlCpuAddr[prgAddr] == 0) {
			_cdlCpuAddr[prgAddr] = cpuAddr;
		}
		_cdlWindows[prgAddr] |= (uint8_t)(1 << (cpuAddr >> 13));
	}
	void LogEdge(AddressInfo& from, uint16_t fromPc, AddressInfo& to, uint16_t toPc, uint8_t kind);
	uint8_t _prevStackPointer = 0;
	uint32_t _prevProgramCounter = 0;

	unique_ptr<DummyNesCpu> _dummyCpu;

	string _cdlFile;

	bool IsRegister(MemoryOperationInfo& op);
	__forceinline void ProcessCallStackUpdates(AddressInfo& destAddr, uint16_t destPc, uint8_t sp);

public:
	NesDebugger(Debugger* debugger);
	~NesDebugger();

	void OnBeforeBreak(CpuType cpuType) override;

	void Reset() override;

	uint64_t GetCpuCycleCount(bool forProfiler = false) override;
	void ResetPrevOpCode() override;

	void ProcessInstruction();

	//[lab fork] lite mode (#14, MesenServer --debug): keep CDL, banked CDL, call edges, call stack and script
	//callbacks; skip access counters, the event log, disassembly-cache upkeep, uninit-read warnings, PPU-side
	//break checks, and CPU break checks while no breakpoint, step or break request is pending
	bool LabLite = false;
	__forceinline bool NeedBreakChecks();

	//[lab fork] #9/#10 exports (little-endian records, see MesenServer)
	string GetCdlCpuMap();
	string GetCallEdges();
	void ResetBankLog();
	void ProcessRead(uint32_t addr, uint8_t value, MemoryOperationType type);
	void ProcessWrite(uint32_t addr, uint8_t value, MemoryOperationType type);
	void ProcessInterrupt(uint32_t originalPc, uint32_t currentPc, bool forNmi) override;
	void ProcessPpuRead(uint16_t addr, uint8_t value, MemoryType memoryType, MemoryOperationType opType);
	void ProcessPpuWrite(uint16_t addr, uint8_t value, MemoryType memoryType);
	void ProcessPpuCycle();

	void ProcessInputOverrides(DebugControllerState inputOverrides[8]) override;

	void Run() override;
	void Step(int32_t stepCount, StepType type) override;
	StepBackConfig GetStepBackConfig() override;

	void DrawPartialFrame() override;

	DebuggerFeatures GetSupportedFeatures() override;
	void SetProgramCounter(uint32_t addr, bool updateDebuggerOnly = false) override;
	uint32_t GetProgramCounter(bool getInstPc) override;

	BreakpointManager* GetBreakpointManager() override;
	ITraceLogger* GetTraceLogger() override;
	PpuTools* GetPpuTools() override;
	ISerializable* GetSerializableCpu() override;
	bool SaveRomToDisk(string filename, bool saveAsIps, CdlStripOption stripOption);
	void GetRomHeader(uint8_t* headerData, uint32_t& size) override;
	CallstackManager* GetCallstackManager() override;
	IAssembler* GetAssembler() override;
	BaseEventManager* GetEventManager() override;

	BaseState& GetState() override;
	void GetPpuState(BaseState& state) override;
	void SetPpuState(BaseState& state) override;
};

namespace NesCdlFlags
{
	enum NesCdlFlags : uint8_t
	{
		PcmData = 0x80
	};
}
