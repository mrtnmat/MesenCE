//[lab fork] MesenServer: headless, persistent NES emulator process driven over stdin/stdout.
//
//  MesenServer.exe ROM [--access observation|full] [--debug | --debug-full] [--mapper-power-on N] [--home DIR]
//
//--debug: lite NES debugger (CDL, cdlmap, edges, call stack, Lua callbacks; no access counters/event log, see
//NesDebugger::LabLite). --debug-full: Mesen's whole debugger. --mapper-power-on N: every mapper power-on byte = N
//(BaseMapper::LabPowerOnByte) instead of the mapper's default.
//
//All protocol work runs on the emulation thread, inside INativeHook::OnFrame (a frame boundary,
//which is also an instruction boundary), so ops can read/write memory and save/load states directly.
//
//Framing (little-endian): request = u32 len + body, response = u32 len + body.
//Request body: u8 cmd, then:
//  'B' batch: a stream of ops (see Op codes below) -> records, ending with a DONE record
//  'P' palette: -> 512 x u32 ARGB (Mesen's default NES palette, index = emphasis<<6 | color)
//  'Q' quit
//Response record: u8 kind, u32 op index, u32 len, payload.
//A failing op produces an ERROR record and aborts the rest of the batch (the server stays up).
#include "Core/Shared/Emulator.h"
#include "Core/Shared/EmuSettings.h"
#include "Core/Shared/SettingTypes.h"
#include "Core/Shared/KeyManager.h"
#include "Core/Shared/BatteryManager.h"
#include "Core/Shared/SaveStateManager.h"
#include "Core/Shared/BaseControlManager.h"
#include "Core/Shared/BaseControlDevice.h"
#include "Core/Shared/DebuggerRequest.h"
#include "Core/Shared/MemoryOperationType.h"
#include "Core/Shared/Interfaces/IInputProvider.h"
#include "Core/Shared/Interfaces/INativeHook.h"
#include "Core/Shared/Video/VideoDecoder.h"
#include "Core/NES/NesConsole.h"
#include "Core/NES/BaseNesPpu.h"
#include "Core/NES/BaseMapper.h"
#include "Core/NES/NesMemoryManager.h"
#include "Core/NES/NesTypes.h"
#include "Core/NES/NesDefaultVideoFilter.h"
#include "Core/NES/NesSoundMixer.h"
#include "Core/NES/GameDatabase.h"
#include "Core/NES/Input/NesController.h"
#include "Core/Debugger/Debugger.h"
#include "Core/Debugger/CdlManager.h"
#include "Core/Debugger/ScriptManager.h"
#include "Core/NES/Debugger/NesDebugger.h"
#include "Utilities/FolderUtilities.h"
#include "Utilities/VirtualFile.h"
#include "Utilities/magic_enum.hpp"
#include <cstdio>
#include <cstring>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <stdexcept>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

using std::string;
using std::vector;

enum Op : uint8_t
{
	OpStep = 0x01,      //u32 n, u8 pad1, u8 pad2: hold buttons for n frames
	OpInputs = 0x02,    //u32 n, n x u8: one pad-1 byte per frame (tape playback)
	OpSettle = 0x03,    //u32 quiet, u32 max: no input until nt+pal+oam unchanged for `quiet` frames -> SETTLE
	OpUntil = 0x04,     //str mem, u32 addr, u8 cmp, u8 value, u8 mask, u32 max, u8 pad1 -> UNTIL
	OpObs = 0x05,       //u8 flags (1 chr, 2 screen, 4 apu) -> OBS [+CHR, CHRBANKS, PPULOG] [+SCREEN] [+APU]
	OpSave = 0x06,      //u16 slot (0xFFFF = none), u8 return blob -> [STATE] (Mesen .mss format, loadable by the GUI)
	OpLoad = 0x07,      //u16 slot
	OpLoadBlob = 0x08,  //u32 len, bytes
	OpReset = 0x09,     //back to the power-on state
	OpHash = 0x0A,      //-> HASH (u64 of nt+pal+oam)
	OpPeek = 0x0B,      //str mem, u32 addr, u32 len: like READ but limited to the observation memory types
	//full access only
	OpRead = 0x10,      //str mem, u32 addr, u32 len -> MEM
	OpWrite = 0x11,     //str mem, u32 addr, u32 len, bytes
	OpCdl = 0x12,       //-> MEM (PRG-ROM CDL flags)
	OpLua = 0x13,       //u32 len, code -> LUA (first return value, tostring'd, binary-safe)
	OpCdlMap = 0x14,    //-> MEM: PRG size x u16 first CPU address (0 = never), then PRG size x u8 mask of 8KB CPU windows
	OpEdges = 0x15,     //-> MEM: n x (u8 kind, u8 fromType, i32 fromAbs, u16 fromCpu, u8 toType, i32 toAbs, u16 toCpu)
	OpMapper = 0x16,    //-> LUA: "name=value\n" mapper state entries
};

enum Rec : uint8_t
{
	RecDone = 0,      //u32 frames run by this batch
	RecError = 1,     //message
	RecObs = 2,       //u32 frame, nt[4096], pal[32], oam[256], ppu regs (see WritePpuRegs)
	RecChr = 3,       //8192
	RecChrBanks = 4,  //n x (u16 base, 4096 bytes)
	RecPpuLog = 5,    //u16 t, u8 x, u8 w, then n x (i16 scanline, u16 dot, u8 kind 0=w 1=r, u8 reg, u8 value)
	RecScreen = 6,    //240x256 x u16 raw PPU output (color | emphasis<<6)
	RecApu = 7,       //n x (u32 frame, u8 reg, u8 value)
	RecSettle = 8,    //u32 frames, u8 settled
	RecState = 9,     //blob
	RecMem = 10,
	RecLua = 11,
	RecUntil = 12,    //u32 frames, u8 hit
	RecHash = 13,     //u64
};

static FILE* _protoOut = nullptr;

static bool ReadExact(void* buf, size_t len)
{
	uint8_t* p = (uint8_t*)buf;
	while(len > 0) {
		size_t n = fread(p, 1, len, stdin);
		if(n == 0) {
			return false;
		}
		p += n;
		len -= n;
	}
	return true;
}

class Reader
{
	const string& _s;
	size_t _pos = 0;
public:
	Reader(const string& s, size_t pos) : _s(s), _pos(pos) {}
	bool Eof() { return _pos >= _s.size(); }
	void Need(size_t n) { if(_pos + n > _s.size()) throw std::runtime_error("truncated op"); }
	uint8_t U8() { Need(1); return (uint8_t)_s[_pos++]; }
	uint16_t U16() { Need(2); uint16_t v; memcpy(&v, &_s[_pos], 2); _pos += 2; return v; }
	uint32_t U32() { Need(4); uint32_t v; memcpy(&v, &_s[_pos], 4); _pos += 4; return v; }
	string Bytes(size_t n) { Need(n); string v = _s.substr(_pos, n); _pos += n; return v; }
	string Str() { return Bytes(U8()); }
};

struct OpData
{
	uint8_t Code = 0;
	uint32_t Index = 0;
	uint32_t A = 0, B = 0, C = 0, D = 0;
	MemoryType Mem = MemoryType::None;
	string Data;
};

class LabServer : public INativeHook, public IInputProvider
{
	Emulator* _emu;
	bool _full;
	bool _debug;
	bool _debugFull;

	NesConsole* _console = nullptr;
	string _out;
	vector<OpData> _ops;
	size_t _pc = 0;
	bool _inBatch = false;
	bool _opStarted = false;
	uint32_t _batchFrames = 0;
	uint8_t _pad[2] = {};

	//current-op progress
	uint32_t _left = 0, _n = 0, _quiet = 0;
	uint64_t _lastHash = 0;

	std::unordered_map<uint32_t, string> _slots;
	string _powerState;

	//logs
	string _apuLog;
	bool _chrArmed = false;
	string _ppuLog;
	string _chrBanks;
	std::unordered_set<uint64_t> _chrSeen;
	int32_t _scriptId = -1;

public:
	LabServer(Emulator* emu, bool full, bool debug, bool debugFull) : _emu(emu), _full(full), _debug(debug), _debugFull(debugFull) {}

	//---- IInputProvider
	bool SetInput(BaseControlDevice* device) override
	{
		uint8_t port = device->GetPort();
		if(port > 1) {
			return false;
		}
		uint8_t v = _pad[port];
		//pad byte bits (NES shift-register order): A B Select Start Up Down Left Right
		device->SetBitValue(NesController::Buttons::A, v & 0x01);
		device->SetBitValue(NesController::Buttons::B, v & 0x02);
		device->SetBitValue(NesController::Buttons::Select, v & 0x04);
		device->SetBitValue(NesController::Buttons::Start, v & 0x08);
		device->SetBitValue(NesController::Buttons::Up, v & 0x10);
		device->SetBitValue(NesController::Buttons::Down, v & 0x20);
		device->SetBitValue(NesController::Buttons::Left, v & 0x40);
		device->SetBitValue(NesController::Buttons::Right, v & 0x80);
		return true;
	}

	//---- memory hooks
	void OnCpuWrite(uint32_t addr, uint8_t value, MemoryOperationType opType) override
	{
		if(addr >= 0x4000 && addr <= 0x4017) {
			if(addr != 0x4014 && addr != 0x4016) {
				Put32(_apuLog, _batchFrames);
				_apuLog.push_back((char)(addr & 0xFF));
				_apuLog.push_back((char)value);
			}
		} else if(_chrArmed && addr >= 0x2000 && addr < 0x4000) {
			LogPpuReg(0, addr & 7, value);
		}
	}

	void OnCpuRead(uint32_t addr, uint8_t value, MemoryOperationType opType) override
	{
		if(_chrArmed && opType == MemoryOperationType::Read && addr >= 0x2000 && addr < 0x4000 && (addr & 7) == 2) {
			LogPpuReg(1, 2, value);
		}
	}

	void OnPpuRead(uint32_t addr, uint8_t value, MemoryOperationType opType) override
	{
		//MMC2/MMC4 latch tiles: a fetch of $xFD8-$xFEF switches the bank of that half
		uint32_t a = addr & 0x0FFF;
		if(a < 0xFD8 || a > 0xFEF) {
			return;
		}
		uint16_t base = (addr & 0x1000);
		BaseMapper* mapper = _console->GetMapper();
		uint8_t bank[4096];
		for(int i = 0; i < 4096; i++) {
			bank[i] = mapper->DebugReadVram(base + i);
		}
		uint64_t sig = Fnv(bank, 4096, 1469598103934665603ULL ^ base);
		if(_chrSeen.insert(sig).second) {
			Put16(_chrBanks, base);
			_chrBanks.append((char*)bank, 4096);
		}
	}

	void LogPpuReg(uint8_t kind, uint8_t reg, uint8_t value)
	{
		BaseNesPpu* ppu = _console->GetPpu();
		Put16(_ppuLog, (uint16_t)(int16_t)ppu->GetCurrentScanline());
		Put16(_ppuLog, (uint16_t)ppu->GetCurrentCycle());
		_ppuLog.push_back((char)kind);
		_ppuLog.push_back((char)reg);
		_ppuLog.push_back((char)value);
	}

	//---- frame hook
	bool OnFrame(bool beforeFirstFrame) override
	{
		if(beforeFirstFrame) {
			_console = dynamic_cast<NesConsole*>(_emu->GetConsoleUnsafe());
			if(!_console) {
				fprintf(stderr, "MesenServer: not an NES ROM\n");
				_exit(4);
			}
			_emu->RegisterInputProvider(this);
			if(_debug) {
				Debugger* dbg = _emu->GetDebugger(true).GetDebugger();
				NesDebugger* nesDbg = dbg ? dynamic_cast<NesDebugger*>(dbg->GetCpuDebugger(CpuType::Nes)) : nullptr;
				if(nesDbg) {
					nesDbg->LabLite = !_debugFull;
				}
				_emu->LabSkipPpuCycle = !_debugFull;
			}
			//LoadRom's RewindManager::InitHistory clears MaximumSpeed: set it here, after the load
			_emu->GetSettings()->SetFlag(EmulationFlags::MaximumSpeed);
			_powerState = SaveBlob();
			_emu->HookCpuWrite = true;
		} else {
			_batchFrames++;
		}

		try {
			bool ticked = !beforeFirstFrame;
			while(true) {
				if(!_inBatch) {
					if(!ReadRequest()) {
						continue;
					}
				}
				if(RunOps(ticked)) {
					return true;
				}
				ticked = false;
				FinishBatch(nullptr);
			}
		} catch(std::exception& ex) {
			FinishBatch(ex.what());
			//keep serving: the emulator stays at this frame boundary
			return OnFrameIdle();
		}
	}

	bool OnFrameIdle()
	{
		//after an error: wait for the next batch at the same boundary
		while(true) {
			try {
				if(!_inBatch && !ReadRequest()) {
					continue;
				}
				if(RunOps(false)) {
					return true;
				}
				FinishBatch(nullptr);
			} catch(std::exception& ex) {
				FinishBatch(ex.what());
			}
		}
	}

private:
	static uint64_t Fnv(const uint8_t* p, size_t n, uint64_t h = 1469598103934665603ULL)
	{
		for(size_t i = 0; i < n; i++) {
			h = (h ^ p[i]) * 1099511628211ULL;
		}
		return h;
	}

	static void Put16(string& s, uint16_t v) { s.append((char*)&v, 2); }
	static void Put32(string& s, uint32_t v) { s.append((char*)&v, 4); }

	void Record(Rec kind, uint32_t index, const void* data, size_t len)
	{
		_out.push_back((char)kind);
		Put32(_out, index);
		Put32(_out, (uint32_t)len);
		_out.append((const char*)data, len);
	}
	void Record(Rec kind, uint32_t index, const string& s) { Record(kind, index, s.data(), s.size()); }

	void Send(const string& body)
	{
		uint32_t len = (uint32_t)body.size();
		fwrite(&len, 4, 1, _protoOut);
		fwrite(body.data(), 1, body.size(), _protoOut);
		fflush(_protoOut);
	}

	bool ReadRequest()
	{
		uint32_t len;
		if(!ReadExact(&len, 4)) {
			_exit(0);
		}
		string body(len, '\0');
		if(len && !ReadExact(&body[0], len)) {
			_exit(0);
		}
		if(body.empty()) {
			throw std::runtime_error("empty request");
		}
		switch(body[0]) {
			case 'Q': _exit(0);
			case 'P': {
				uint32_t pal[512];
				NesDefaultVideoFilter::GetFullPalette(pal, _emu->GetSettings()->GetNesConfig(), _console->GetPpu()->GetPpuModel());
				Send(string((char*)pal, sizeof(pal)));
				return false;
			}
			case 'B': {
				ParseOps(body);
				_inBatch = true;
				_pc = 0;
				_opStarted = false;
				_batchFrames = 0;
				_apuLog.clear();
				_out.clear();
				return true;
			}
			default: throw std::runtime_error("unknown command");
		}
	}

	void FinishBatch(const char* error)
	{
		if(error) {
			Record(RecError, _pc < _ops.size() ? _ops[_pc].Index : 0, string(error));
			_pad[0] = _pad[1] = 0;
			Disarm();
		}

		Record(RecDone, (uint32_t)_ops.size(), &_batchFrames, 4);
		Send(_out);
		_out.clear();
		_inBatch = false;
		_ops.clear();
	}

	MemoryType ParseMem(const string& name)
	{
		auto t = magic_enum::enum_cast<MemoryType>(name, magic_enum::case_insensitive);
		if(!t.has_value()) {
			throw std::runtime_error("unknown memType " + name);
		}
		return t.value();
	}

	void ParseOps(const string& body)
	{
		_ops.clear();
		Reader r(body, 1);
		uint32_t index = 0;
		while(!r.Eof()) {
			OpData op;
			op.Code = r.U8();
			op.Index = index++;
			switch(op.Code) {
				case OpStep: op.A = r.U32(); op.B = r.U8(); op.C = r.U8(); break;
				case OpInputs: op.A = r.U32(); op.Data = r.Bytes(op.A); break;
				case OpSettle: op.A = r.U32(); op.B = r.U32(); break;
				case OpUntil:
					op.Mem = ParseMem(r.Str()); op.A = r.U32(); op.B = r.U8(); op.C = r.U8(); op.D = r.U8();
					op.Data = r.Bytes(5); //u32 max, u8 pad1
					break;
				case OpObs: op.A = r.U8(); break;
				case OpSave: op.A = r.U16(); op.B = r.U8(); break;
				case OpLoad: op.A = r.U16(); break;
				case OpLoadBlob: op.Data = r.Bytes(r.U32()); break;
				case OpReset: case OpHash: case OpCdl: case OpCdlMap: case OpEdges: case OpMapper: break;
				case OpPeek: case OpRead: op.Mem = ParseMem(r.Str()); op.A = r.U32(); op.B = r.U32(); break;
				case OpWrite: op.Mem = ParseMem(r.Str()); op.A = r.U32(); op.Data = r.Bytes(r.U32()); break;
				case OpLua: op.Data = r.Bytes(r.U32()); break;
				default: throw std::runtime_error("unknown op code " + std::to_string(op.Code));
			}
			if(op.Code >= 0x10 && !_full) {
				throw std::runtime_error("op " + std::to_string(op.Code) + " refused: needs full access");
			}
			_ops.push_back(std::move(op));
		}
	}

	//---- memory access
	static bool IsObservable(MemoryType t, uint32_t addr, uint32_t len)
	{
		switch(t) {
			case MemoryType::NesPpuMemory: return addr + len <= 0x3000;
			case MemoryType::NesPaletteRam: case MemoryType::NesSpriteRam: case MemoryType::NesNametableRam: return true;
			default: return false;
		}
	}

	uint32_t MemSize(MemoryType t)
	{
		switch(t) {
			case MemoryType::NesMemory: return 0x10000;
			case MemoryType::NesPpuMemory: return 0x4000;
			default: return _emu->GetMemory(t).Size;
		}
	}

	uint8_t Peek(MemoryType t, uint32_t a)
	{
		switch(t) {
			case MemoryType::NesMemory: return _console->DebugRead((uint16_t)a);
			case MemoryType::NesPpuMemory: return _console->GetMapper()->DebugReadVram((uint16_t)a);
			default: {
				ConsoleMemoryInfo m = _emu->GetMemory(t);
				return (m.Memory && a < m.Size) ? ((uint8_t*)m.Memory)[a] : 0;
			}
		}
	}

	void Poke(MemoryType t, uint32_t a, uint8_t v)
	{
		switch(t) {
			case MemoryType::NesMemory: _console->GetMemoryManager()->DebugWrite((uint16_t)a, v, true); break;
			case MemoryType::NesPpuMemory: _console->GetMapper()->DebugWriteVram((uint16_t)a, v); break;
			default: {
				ConsoleMemoryInfo m = _emu->GetMemory(t);
				if(!m.Memory || a >= m.Size) {
					throw std::runtime_error("write out of range");
				}
				((uint8_t*)m.Memory)[a] = v;
			}
		}
	}

	string ReadRange(MemoryType t, uint32_t addr, uint32_t len)
	{
		uint32_t size = MemSize(t);
		if((uint64_t)addr + len > size) {
			throw std::runtime_error("read out of range (size " + std::to_string(size) + ")");
		}
		string s(len, '\0');
		if(t == MemoryType::NesMemory || t == MemoryType::NesPpuMemory) {
			for(uint32_t i = 0; i < len; i++) {
				s[i] = (char)Peek(t, addr + i);
			}
		} else {
			memcpy(&s[0], (uint8_t*)_emu->GetMemory(t).Memory + addr, len);
		}
		return s;
	}

	uint64_t VisHash()
	{
		string v = ReadRange(MemoryType::NesPpuMemory, 0x2000, 0x1000);
		uint64_t h = Fnv((uint8_t*)v.data(), v.size());
		h = Fnv((uint8_t*)_emu->GetMemory(MemoryType::NesPaletteRam).Memory, 32, h);
		h = Fnv((uint8_t*)_emu->GetMemory(MemoryType::NesSpriteRam).Memory, 256, h);
		return h;
	}

	//---- states
	string SaveBlob()
	{
		std::stringstream ss;
		_emu->Serialize(ss, false, 0);
		return ss.str();
	}

	//Mesen's own savestate format ("MSS" header + screenshot): what the GUI and Lua's loadSavestate read
	string ExportBlob()
	{
		std::stringstream ss;
		_emu->GetSaveStateManager()->SaveState(ss);
		return ss.str();
	}

	//accepts both an exported MSS state and a raw slot blob
	void LoadBlob(const string& blob)
	{
		std::istringstream in(blob);
		bool ok;
		if(blob.size() >= 3 && memcmp(blob.data(), "MSS", 3) == 0) {
			ok = _emu->GetSaveStateManager()->LoadState(in);
		} else {
			ok = _emu->Deserialize(in, SaveStateManager::FileFormatVersion, false) == DeserializeResult::Success;
		}
		if(!ok) {
			throw std::runtime_error("state load failed");
		}
		_apuLog.clear();
	}

	//---- observation
	void WritePpuRegs(string& s)
	{
		NesPpuState st;
		_console->GetPpu()->GetState(st);
		s.push_back((char)st.ScrollX);
		Put16(s, st.TmpVideoRamAddr);
		Put16(s, st.VideoRamAddr);
		Put16(s, st.Control.BackgroundPatternAddr);
		Put16(s, st.Control.SpritePatternAddr);
		s.push_back((char)st.Control.LargeSprites);
		s.push_back((char)st.Mask.BackgroundEnabled);
		s.push_back((char)st.Mask.SpritesEnabled);
		s.push_back((char)st.Mask.BackgroundMask);
		s.push_back((char)st.Mask.SpriteMask);
		s.push_back((char)st.Mask.Grayscale);
		Put32(s, st.FrameCount);
		Put32(s, (uint32_t)st.Scanline);
		Put32(s, st.Cycle);
		s.push_back((char)st.WriteToggle);
		s.push_back((char)st.Control.NmiOnVerticalBlank);
		s.push_back((char)st.Control.VerticalWrite);
	}

	void Arm()
	{
		NesPpuState st;
		_console->GetPpu()->GetState(st);
		_ppuLog.clear();
		Put16(_ppuLog, st.TmpVideoRamAddr);
		_ppuLog.push_back((char)st.ScrollX);
		_ppuLog.push_back((char)st.WriteToggle);
		_chrBanks.clear();
		_chrSeen.clear();
		_chrArmed = true;
		_emu->HookCpuRead = true;
		_emu->HookPpuRead = true;
	}

	void Disarm()
	{
		_chrArmed = false;
		_emu->HookCpuRead = false;
		_emu->HookPpuRead = false;
	}

	void Observe(const OpData& op)
	{
		string s;
		Put32(s, _batchFrames);
		s += ReadRange(MemoryType::NesPpuMemory, 0x2000, 0x1000);
		s.append((char*)_emu->GetMemory(MemoryType::NesPaletteRam).Memory, 32);
		s.append((char*)_emu->GetMemory(MemoryType::NesSpriteRam).Memory, 256);
		WritePpuRegs(s);
		Record(RecObs, op.Index, s);
		if(op.A & 1) {
			Record(RecChr, op.Index, ReadRange(MemoryType::NesPpuMemory, 0, 0x2000));
			Record(RecChrBanks, op.Index, _chrBanks);
			Record(RecPpuLog, op.Index, _ppuLog);
		}
		if(op.A & 2) {
			uint16_t* buf = _console->GetPpu()->GetScreenBuffer(false, false);
			Record(RecScreen, op.Index, buf, 256 * 240 * 2);
		}
		if(op.A & 4) {
			Record(RecApu, op.Index, _apuLog);
			_apuLog.clear();
		}
	}

	bool CheckUntil(const OpData& op)
	{
		uint8_t v = Peek(op.Mem, op.A) & op.D;
		switch(op.B) {
			case 0: return v == op.C;
			case 1: return v != op.C;
			case 2: return v > op.C;
			case 3: return v < op.C;
			default: throw std::runtime_error("bad cmp");
		}
	}

	Debugger* GetDebugger()
	{
		Debugger* dbg = _emu->GetDebugger(false).GetDebugger();
		if(!dbg) {
			throw std::runtime_error("debugger not active (start the server with --debug)");
		}
		return dbg;
	}

	//Advance the op list at a frame boundary. `ticked` = one frame has run since the last call.
	//Returns true while the current op needs more frames.
	bool RunOps(bool ticked)
	{
		while(_pc < _ops.size()) {
			OpData& op = _ops[_pc];
			switch(op.Code) {
				case OpStep:
					if(!_opStarted) { _opStarted = true; _left = op.A; _pad[0] = (uint8_t)op.B; _pad[1] = (uint8_t)op.C; }
					else if(ticked) { _left--; ticked = false; }
					if(_left > 0) return true;
					break;

				case OpInputs:
					if(!_opStarted) { _opStarted = true; _n = 0; }
					else if(ticked) { _n++; ticked = false; }
					if(_n < op.A) { _pad[0] = (uint8_t)op.Data[_n]; _pad[1] = 0; return true; }
					break;

				case OpSettle:
					if(!_opStarted) { _opStarted = true; _n = 0; _quiet = 0; _lastHash = VisHash(); _pad[0] = _pad[1] = 0; }
					else if(ticked) {
						ticked = false;
						_n++;
						uint64_t h = VisHash();
						_quiet = (h == _lastHash) ? _quiet + 1 : 0;
						_lastHash = h;
					}
					if(_quiet < op.A && _n < op.B) return true;
					{
						string s;
						Put32(s, _n);
						s.push_back((char)(_quiet >= op.A));
						Record(RecSettle, op.Index, s);
					}
					break;

				case OpUntil: {
					uint32_t max; memcpy(&max, op.Data.data(), 4);
					if(!_opStarted) {
						if(!_full && !IsObservable(op.Mem, op.A, 1)) throw std::runtime_error("until: memType needs full access");
						_opStarted = true; _n = 0; _pad[0] = (uint8_t)op.Data[4]; _pad[1] = 0;
					} else if(ticked) { _n++; ticked = false; }
					bool hit = CheckUntil(op);
					if(!hit && _n < max) return true;
					string s;
					Put32(s, _n);
					s.push_back((char)hit);
					Record(RecUntil, op.Index, s);
					break;
				}

				case OpObs:
					if(op.A & 1) {
						//two-phase: arm the CHR-latch and PPU-register capture, run one frame with no input
						if(!_opStarted) { _opStarted = true; _pad[0] = _pad[1] = 0; Arm(); return true; }
						if(ticked) ticked = false;
						Disarm();
					}
					Observe(op);
					break;

				case OpSave: {
					if(op.B) Record(RecState, op.Index, ExportBlob());
					if(op.A != 0xFFFF) _slots[op.A] = SaveBlob();
					break;
				}
				case OpLoad: {
					auto it = _slots.find(op.A);
					if(it == _slots.end()) throw std::runtime_error("empty slot " + std::to_string(op.A));
					LoadBlob(it->second);
					break;
				}
				case OpLoadBlob: LoadBlob(op.Data); break;
				case OpReset: LoadBlob(_powerState); break;
				case OpHash: { uint64_t h = VisHash(); Record(RecHash, op.Index, &h, 8); break; }

				case OpPeek:
					if(!IsObservable(op.Mem, op.A, op.B)) throw std::runtime_error("peek: memType/range not observable");
					Record(RecMem, op.Index, ReadRange(op.Mem, op.A, op.B));
					break;
				case OpRead: Record(RecMem, op.Index, ReadRange(op.Mem, op.A, op.B)); break;
				case OpWrite:
					for(size_t i = 0; i < op.Data.size(); i++) Poke(op.Mem, op.A + (uint32_t)i, (uint8_t)op.Data[i]);
					break;

				case OpCdl: {
					Debugger* dbg = GetDebugger();
					uint32_t size = _emu->GetMemory(MemoryType::NesPrgRom).Size;
					string s(size, '\0');
					dbg->GetCdlManager()->GetCdlData(0, size, MemoryType::NesPrgRom, (uint8_t*)&s[0]);
					Record(RecMem, op.Index, s);
					break;
				}
				case OpCdlMap: Record(RecMem, op.Index, GetCdlMap()); break;
				case OpEdges: Record(RecMem, op.Index, GetEdges()); break;
				case OpMapper: Record(RecLua, op.Index, GetMapperState()); break;

				case OpLua: {
					ScriptManager* sm = GetDebugger()->GetScriptManager();
					if(_scriptId < 0) {
						_scriptId = sm->LoadScript("lab", "", "", -1);
					}
					bool ok = false;
					string r = sm->Eval(_scriptId, op.Data, ok);
					if(!ok) throw std::runtime_error("lua: " + r);
					Record(RecLua, op.Index, r);
					break;
				}
			}
			_pc++;
			_opStarted = false;
			_pad[0] = _pad[1] = 0;
		}
		return false;
	}

	string GetCdlMap();
	string GetEdges();

	//"tab-separated: address, name, value" per mapper state entry, then "cpu\t$XXXX\tmemType:offset" per 8KB window
	string GetMapperState()
	{
		string s;
		CartridgeState cart = _console->GetMapper()->GetState();
		for(uint32_t i = 0; i < cart.CustomEntryCount; i++) {
			MapperStateEntry& e = cart.CustomEntries[i];
			string value;
			switch(e.Type) {
				case MapperStateValueType::None: break;
				case MapperStateValueType::String: value = (char*)e.Value; break;
				case MapperStateValueType::Bool: value = e.Value[0] ? "true" : "false"; break;
				default: {
					int64_t v = 0;
					for(int i = 7; i >= 0; i--) v = (v << 8) | e.Value[i];
					value = std::to_string(v);
					break;
				}
			}
			s += string((char*)e.Address) + "\t" + string((char*)e.Name) + "\t" + value + "\n";
		}
		for(uint32_t a = 0x6000; a < 0x10000; a += 0x2000) {
			AddressInfo rel = { (int32_t)a, MemoryType::NesMemory };
			AddressInfo abs = _console->GetAbsoluteAddress(rel);
			char buf[96];
			snprintf(buf, sizeof(buf), "cpu\t$%04X\t%s:%d\n", a, abs.Address < 0 ? "none" : string(magic_enum::enum_name(abs.Type)).c_str(), abs.Address);
			s += buf;
		}
		return s;
	}
};

static NesDebugger* GetNesDebugger(Debugger* dbg)
{
	NesDebugger* nesDbg = dynamic_cast<NesDebugger*>(dbg->GetCpuDebugger(CpuType::Nes));
	if(!nesDbg) {
		throw std::runtime_error("NES debugger not available");
	}
	return nesDbg;
}

string LabServer::GetCdlMap()
{
	return GetNesDebugger(GetDebugger())->GetCdlCpuMap();
}

string LabServer::GetEdges()
{
	return GetNesDebugger(GetDebugger())->GetCallEdges();
}

int main(int argc, char* argv[])
{
	string rom;
	string home;
	bool full = false;
	bool debug = false;
	bool debugFull = false;
	for(int i = 1; i < argc; i++) {
		string a = argv[i];
		if(a == "--access" && i + 1 < argc) {
			full = string(argv[++i]) == "full";
		} else if(a == "--debug") {
			debug = true;
		} else if(a == "--debug-full") {
			debug = debugFull = true;
		} else if(a == "--mapper-power-on" && i + 1 < argc) {
			BaseMapper::LabPowerOnByte = std::stoi(argv[++i], nullptr, 0) & 0xFF;
		} else if(a == "--home" && i + 1 < argc) {
			home = argv[++i];
		} else {
			rom = a;
		}
	}
	if(rom.empty()) {
		fprintf(stderr, "usage: MesenServer ROM [--access observation|full] [--debug | --debug-full] [--mapper-power-on N] [--home DIR]\n");
		return 2;
	}
	if(debug && !full) {
		fprintf(stderr, "--debug needs --access full\n");
		return 2;
	}

#ifdef _WIN32
	_setmode(_fileno(stdin), _O_BINARY);
	//protocol goes to the original stdout; anything else printing to stdout (Lua print) lands on stderr
	int protoFd = _dup(_fileno(stdout));
	_dup2(_fileno(stderr), _fileno(stdout));
	_setmode(protoFd, _O_BINARY);
	_protoOut = _fdopen(protoFd, "wb");
#else
	int protoFd = dup(fileno(stdout));
	dup2(fileno(stderr), fileno(stdout));
	_protoOut = fdopen(protoFd, "wb");
#endif

	if(home.empty()) {
		home = FolderUtilities::CombinePath(FolderUtilities::GetFolderName(rom), ".mesenserver");
	}
	FolderUtilities::SetHomeFolder(home);

	//the game DB (MesenNesDB.txt) is read from the home folder, which is a fresh dir here: load the one next to the exe
	string dbPath = FolderUtilities::CombinePath(FolderUtilities::GetFolderName(argv[0]), "MesenNesDB.txt");
	if(!GameDatabase::LabInitFrom(dbPath)) {
		fprintf(stderr, "MesenServer: warning: no game database at %s (header-only ROM info)\n", dbPath.c_str());
	}

	unique_ptr<Emulator> emu(new Emulator());
	emu->Initialize(false);
	KeyManager::SetSettings(emu->GetSettings());

	EmuSettings* settings = emu->GetSettings();
	NesConfig& nes = settings->GetNesConfig();
	nes.RamPowerOnState = RamState::AllZeros;
	nes.RandomizeMapperPowerOnState = false;
	nes.RandomizeCpuPpuAlignment = false;
	nes.Port1.Type = ControllerType::NesController;
	nes.Port2.Type = ControllerType::NesController;
	nes.EnableHdPacks = false;
	NesDefaultVideoFilter::GetDefaultPalette(nes.UserPalette);
	settings->GetAudioConfig().EnableAudio = false;
	settings->GetEmulationConfig().RunAheadFrames = 0;
	settings->GetDebugConfig().ScriptAllowIoOsAccess = true;
	settings->GetDebugConfig().ScriptTimeout = 3600;
	settings->GetPreferences().DisableGameSelectionScreen = true;
	settings->GetPreferences().RewindBufferSize = 0;
	settings->SetFlag(EmulationFlags::MaximumSpeed);

	emu->GetBatteryManager()->SetDisabled(true);
	emu->GetVideoDecoder()->SetDisabled(true);
	NesSoundMixer::LabDisabled = true;

	LabServer server(emu.get(), full, debug, debugFull);
	emu->SetNativeHook(&server);

	if(!emu->LoadRom((VirtualFile)rom, VirtualFile())) {
		fprintf(stderr, "MesenServer: could not load %s\n", rom.c_str());
		return 3;
	}

	//all work happens on the emulation thread (LabServer::OnFrame); it exits the process on 'Q' or EOF
	while(emu->IsRunning()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return 0;
}
