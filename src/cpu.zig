//! Game Boy DMG CPU Emulator
//!
//! This file implements a cycle-accurate emulation of the Game Boy DMG CPU, which is based on
//! the Sharp LR35902 processor (a modified Z80-like design). The emulation models both the
//! internal CPU state and the external pin interface to accurately simulate timing and behavior.
//!
//! ## Architecture Overview
//!
//! The emulator is structured around several key components:
//!
//! 1. **Pins**: Models the physical CPU pins (address bus, data bus, control signals)
//! 2. **State**: Contains all internal CPU registers and execution context
//! 3. **Microcode**: Low-level operations that execute over multiple clock cycles
//! 4. **Decoder**: Compile-time generated lookup table mapping opcodes to microcode sequences
//! 5. **Execution Engine**: Manages microcode execution and instruction timing
//!
//! ## Key Concepts
//!
//! ### Microcode Execution
//! Instructions are broken down into sequences of microcode operations. Each microcode operation
//! represents one machine cycle (4 clock ticks) and can modify CPU state and control external pins.
//! This approach allows cycle-accurate timing that matches real hardware.
//!
//! ### Register System
//! The CPU registers are modeled using a flexible type system that allows accessing 16-bit
//! register pairs as either a full word or separate high/low bytes. The AF register pair
//! has special handling for the flags register format.
//!
//! ### Instruction Decoding
//! The decoder uses a compile-time builder pattern to generate efficient lookup tables.
//! Instructions are defined using bit pattern strings (e.g., "01xxxxxx") that specify
//! which opcodes match and what microcode sequence to execute.
//!
//! ### Memory Interface
//! The emulator communicates with external memory through the Pins interface, modeling
//! the actual bus timing and control signals used by the real hardware.
//!
//! ## Usage
//!
//! The main entry point is the `tick()` function, which advances the CPU by one clock cycle.
//! External code is responsible for responding to pin changes to simulate memory and I/O.

const std = @import("std");

/// Represents the physical pins on a Game Boy DMG CPU chip.
/// This packed struct mirrors the actual hardware interface that would be used
/// to communicate with external components like cartridge ROM/RAM and internal components.
pub const Pins = struct {
    /// 16-bit address bus (output from CPU)
    /// Specifies the memory address the CPU wants to access
    addr: u16,

    /// 8-bit data bus (bidirectional)
    /// Carries data between CPU and external components
    data: u8,

    /// Chip select signal (output from CPU)
    /// When true, indicates the CPU is actively using the bus
    cs: bool,

    /// Read enable signal (output from CPU)
    /// When true, CPU is reading data from the address on the bus
    rd: bool,

    /// Write enable signal (output from CPU)
    /// When true, CPU is writing data to the address on the bus
    wr: bool,

    /// Initialize all pins to their default/reset state
    pub const zero = @This(){
        .addr = 0,
        .data = 0,
        .cs = false,
        .rd = false,
        .wr = false,
    };
};

/// Complete CPU state including all registers and execution context.
/// This represents the internal state of the Game Boy DMG processor that would
/// normally be invisible to external components.
pub const State = struct {
    /// Accumulator (A) and flags (F) register pair
    /// The accumulator is the primary register for arithmetic operations
    af: Reg(16),

    /// General purpose register pairs that can be accessed as 16-bit or 8-bit
    bc: Reg(16), // B and C registers
    de: Reg(16), // D and E registers
    hl: Reg(16), // H and L registers (often used for memory addressing)

    /// Program counter - points to the next instruction to execute
    pc: Reg(16),

    /// Stack pointer - points to the top of the stack in memory
    sp: Reg(16),

    /// Interrupt enable register - controls interrupt handling
    ie: Reg(8),

    /// Instruction register - holds the current opcode being executed
    ir: Reg(8),

    /// Temporary register for internal CPU operations
    /// Used as scratch space during multi-cycle operations
    z: Reg(16),

    /// Execution engine state - tracks microcode execution
    exec: Exec,

    /// Default CPU state after reset/power-on
    pub const reset = @This(){
        .af = .zero,
        .bc = .zero,
        .de = .zero,
        .hl = .zero,
        .pc = .zero,
        .sp = .zero,
        .ie = .zero,
        .ir = .zero,
        .z = .zero,
        .exec = .reset,
    };

    /// Creates register types that can be accessed as either 8-bit or 16-bit values.
    /// This models how Game Boy DMG registers can be used in different ways:
    /// - 8-bit registers store single bytes
    /// - 16-bit registers can be accessed as a word or as separate high/low bytes
    pub fn Reg(comptime width: u16) type {
        return switch (width) {
            8 => packed struct {
                /// Single byte value
                b: u8,

                pub const zero = @This(){ .b = 0 };
            },
            16 => packed union {
                /// Access as separate high and low bytes
                b: packed struct {
                    /// Lower 8 bits (least significant byte)
                    l: u8,
                    /// Upper 8 bits (most significant byte)
                    h: u8,
                },

                /// Access as flags register (only valid for AF register)
                /// Game Boy DMG flags are stored in specific bit positions
                f: packed struct {
                    /// Unused bits (always 0)
                    pad: u4,
                    /// Carry flag - set when arithmetic produces carry/borrow
                    c: bool,
                    /// Half-carry flag - used for BCD arithmetic
                    h: bool,
                    /// Add/subtract flag - indicates last operation type (BCD)
                    n: bool,
                    /// Zero flag - set when result is zero
                    z: bool,
                },

                /// Access as full 16-bit word
                w: u16,

                pub const zero = @This(){ .w = 0 };
            },
            else => @compileError(std.fmt.comptimePrint(
                "Expected register width of 8 or 16 but found {}",
                .{width},
            )),
        };
    }

    /// Get a pointer to a specific register byte for reading/writing.
    /// This handles the complexity of accessing different register formats uniformly.
    /// Get a pointer to a register using string-based addressing.
    /// Supports both 8-bit and 16-bit register access with simple dot notation.
    /// Examples: "af", "bc.h", "hl.l", "ir"
    /// Returns *u8 for byte access or *u16 for word access based on the register specification.
    inline fn idx(this: *@This(), comptime reg: []const u8) ret: {
        if (std.mem.indexOfScalar(u8, reg, '.') != null or @FieldType(@This(), reg) == Reg(8)) {
            break :ret *u8;
        } else {
            break :ret *u16;
        }
    } {
        const i = comptime std.mem.indexOfScalar(u8, reg, '.') orelse reg.len;
        return switch (@FieldType(@This(), reg[0..i])) {
            Reg(8) => &@field(this, reg).b, // 8-bit register: return byte pointer
            Reg(16) => blk: {
                if (i != reg.len) {
                    // 16-bit register with half specifier (e.g., "bc.h")
                    break :blk &@field(@field(this, reg[0..i]).b, reg[i + 1 ..]);
                } else {
                    // 16-bit register as word (e.g., "hl")
                    break :blk &@field(this, reg).w;
                }
            },
            else => @compileError(std.fmt.comptimePrint(
                "idx() only supports registers but got {} instead",
                .{reg},
            )),
        };
    }
};

/// Function signature for microcode operations that simulate 1 clock cycle.
/// Each microcode operation receives the CPU state and pin state,
/// allowing it to modify internal registers and control external bus signals.
const Microcode = fn (*State, *Pins) void;

/// Instruction decoder that maps opcodes to microcode sequences.
/// Uses a builder pattern to construct lookup tables at compile time.
const Decoder = struct {
    /// List of instruction encodings, ordered from most specific to least specific.
    /// When decoding, the first matching encoding is used.
    encoders: []const Encoding,

    /// Maps bit patterns to microcode sequences.
    /// Each encoding defines which opcodes it matches and what code to execute.
    const Encoding = struct {
        /// Bitmask indicating which opcodes this encoding handles
        mask: std.bit_set.ArrayBitSet(u64, 256),

        /// Microcode sequence to execute for matching opcodes
        code: []const type,

        /// Create an encoding from a bit pattern string.
        /// Pattern format:
        /// - '0': bit must be 0
        /// - '1': bit must be 1
        /// - 'x': bit can be either 0 or 1 (don't care)
        /// Example: "01xxxxxx" matches opcodes 0x40-0x7F
        fn init(comptime pattern: *const [8]u8, comptime code: []const type) @This() {
            @setEvalBranchQuota(1000000);
            var ones: u8 = 0; // Required '1' bits
            var care: u8 = 0; // Bits we care about (not 'x')

            // Parse the encoding string
            for (pattern, 0..) |c, i| {
                switch (c) {
                    '0', '1', 'x' => {},
                    else => @compileError(std.fmt.comptimePrint(
                        "Expected '0', '1' or 'x' in encoding string but found \"{}\"",
                        .{pattern},
                    )),
                }
                const bit = 1 << 7 - i;
                ones |= if (c == '1') bit else 0;
                care |= if (c == 'x') 0 else bit;
            }

            // Build bitmask of all matching opcodes
            var mask = std.bit_set.ArrayBitSet(u64, 256).initEmpty();
            inline for (0..mask.capacity()) |i| {
                // Opcode matches if (opcode XOR required_ones) AND care_mask == 0
                mask.setValue(i, i ^ ones & care == 0);
            }
            return .{ .mask = mask, .code = code };
        }
    };

    /// Marker type indicating the decoder shouldn't automatically set the execution
    /// stage to the microcode specified by the IR register in the lookup table.
    /// Used for instructions that modify control flow (jumps, calls, etc.)
    const NoIr = struct {};

    /// Base decoder with catch-all NOP instruction
    const init = @This(){
        .encoders = &.{.{
            .mask = .initFull(), // Matches all opcodes
            .code = &.{ struct {
                pub fn op(comptime _: u8, comptime _: u2, _: *State, _: *Pins) void {}
            }, NoIr },
        }},
    };

    /// Add a new instruction encoding to the decoder.
    /// Maintains proper ordering (most specific encodings first).
    fn add(comptime this: @This(), comptime pattern: *const [8]u8, comptime code: anytype) @This() {
        const codes = @as([code.len]type, code);
        const encoding = Encoding.init(pattern, &codes);

        // Find insertion point to maintain specificity order
        const idx = std.sort.lowerBound(Encoding, this.encoders, encoding, struct {
            fn cmp(ctx: Encoding, val: Encoding) std.math.Order {
                return std.math.order(ctx.mask.count(), val.mask.count());
            }
        }.cmp);

        // Insert new encoding at correct position
        var encoders = this.encoders[0..idx] ++ .{encoding} ++
            this.encoders[idx..];
        return .{ .encoders = encoders[0..] };
    }

    /// Build the final opcode lookup table and link all microcode together.
    /// Returns a type containing the completed lookup table.
    /// `nextlut` is the decoder lut that will be used when following the IR pointer. Leave null
    /// to use this decoder lut.
    fn Build(comptime this: @This(), comptime nextlut: ?type) type {
        return struct {
            /// 256-entry lookup table mapping opcodes to microcode entry points.
            /// This is the main interface used by the CPU execution engine.
            pub const lut = compute_lut: {
                @setEvalBranchQuota(1000000);

                // Generate microcode sequences for each encoding
                var code: [this.encoders.len][256]*const Microcode = undefined;
                for (this.encoders, &code) |encoder, *pfn| {
                    pfn.* = link(encoder);
                }

                // Build final lookup table by finding first matching encoding per opcode
                var decoder: [256]*const Microcode = undefined;
                for (0..decoder.len) |byte| {
                    for (this.encoders, code) |encoder, pfn_table| {
                        if (encoder.mask.isSet(byte)) {
                            decoder[byte] = pfn_table[byte];
                            break;
                        }
                    }
                }
                break :compute_lut decoder;
            };

            /// Link microcode stages together into executable sequences.
            /// Each instruction executes as a series of 4-cycle machine operations.
            fn link(comptime encoding: Encoding) [256]*const Microcode {
                const noir = encoding.code[encoding.code.len - 1] == NoIr;
                comptime var state = encoding.code.len - @intFromBool(noir);
                comptime var last = [1]?*const Microcode{null} ** 256;

                // Process each microcode stage (working backwards for linking)
                inline while (state != 0) {
                    state -= 1;
                    comptime var stage = 4;

                    // Each stage has 4 clock cycles
                    inline while (stage != 0) {
                        stage -= 1;
                        inline for (0..last.len) |byte| {
                            if (!encoding.mask.isSet(byte)) {
                                continue; // Skip opcodes this encoding doesn't handle
                            }

                            // Create microcode function for this stage
                            last[byte] = struct {
                                fn op(s: *State, p: *Pins) void {
                                    // Execute this stage
                                    encoding.code[state].op(byte, stage, s, p);

                                    // Link to next stage or set IR-based microcode
                                    if (last[byte]) |pfn| {
                                        s.exec.stage = pfn; // Continue to next stage
                                    } else if (!noir) {
                                        // Auto-set microcode from IR lookup
                                        if (nextlut) |next| {
                                            s.exec.stage = @field(next, lut)[s.ir.b];
                                        } else {
                                            s.exec.stage = lut[s.ir.b];
                                        }
                                    }
                                }
                            }.op;
                        }
                    }
                }

                // Convert nullable pointers to definite pointers
                comptime var final = [1]*const Microcode{undefined} ** 256;
                inline for (&final, last) |*pfn, last_pfn| {
                    if (last_pfn) |final_pfn| {
                        pfn.* = final_pfn;
                    }
                }
                return final;
            }
        };
    }
};

/// Execution engine containing microcode implementations and CPU utilities.
/// This handles the low-level details of instruction execution timing and bus operations.
const Exec = struct {
    /// Current microcode stage being executed
    stage: *const Microcode,

    /// Reset state pointing to NOP instruction microcode
    const reset = @This(){ .stage = decoder.lut[0x00] };

    /// Determine chip select signal based on memory address.
    /// Models Game Boy DMG memory map where certain ranges may be unmapped or special.
    fn cs(addr: u16) bool {
        return switch (addr) {
            0xA000...0xFDFF => false, // Cartridge RAM area (may be disabled)
            else => true, // ROM, WRAM, or I/O
        };
    }

    /// Extract a range of bits from an integer.
    /// Helper function for decoding instruction bit fields.
    inline fn extract(
        from: anytype,
        idx: std.math.Log2Int(@TypeOf(from)),
        cnt: u16,
    ) std.meta.Int(.unsigned, cnt) {
        return @truncate(from >> idx);
    }

    /// Bus read operation microcode generator.
    /// Creates a 4-cycle bus read sequence that loads data into the specified register.
    /// Cycle timing:
    /// 0: Setup read signals
    /// 1: Assert chip select
    /// 2: Wait state
    /// 3: Latch data
    fn ReadBus(comptime reg: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                switch (stage) {
                    0 => {
                        pins.*.rd = true;
                        pins.*.wr = false;
                        pins.*.cs = true;
                    },
                    1 => pins.*.cs = cs(pins.*.addr),
                    2 => {}, // Wait state for memory access
                    3 => r8(reg, ir, state).* = pins.*.data,
                }
            }
        };
    }

    /// Bus write operation microcode generator.
    /// Creates a 4-cycle bus write sequence that outputs data from the specified register.
    /// Cycle timing:
    /// 0: Setup initial signals
    /// 1: Assert chip select
    /// 2: Output data and assert write
    /// 3: Complete write cycle
    fn WriteBus(comptime reg: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                switch (stage) {
                    0 => {
                        pins.*.rd = false;
                        pins.*.wr = false;
                        pins.*.cs = true;
                    },
                    1 => pins.*.cs = cs(pins.*.addr),
                    2 => {
                        pins.*.wr = true;
                        pins.*.data = r8(reg, ir, state).*;
                    },
                    3 => {}, // Complete write cycle
                }
            }
        };
    }

    /// Assert address line with register microcode generator.
    /// Asserts onto the address lines on stage 0 the contents of the register specified.
    /// Set the address bus to the value of a 16-bit register.
    /// Used to prepare for memory operations using register-indirect addressing.
    fn AssertAddrReg(comptime reg: []const u8) type {
        return struct {
            pub fn op(comptime _: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                switch (stage) {
                    0 => pins.addr = state.idx(reg).*,
                    else => {},
                }
            }
        };
    }

    /// Convenience function that combines address setup and bus read in one operation.
    /// Sets the address bus to the value of a 16-bit register, then reads data into an 8-bit reg.
    /// This is a common pattern for memory-indirect operations like LD r8,(HL).
    fn ReadBusReg(comptime addr: []const u8, comptime reg: anytype) type {
        return Join(.{ AssertAddrReg(addr), ReadBus(reg) });
    }

    /// Convenience function that combines address setup and bus write in one operation.
    /// Sets the address bus to the value of a 16-bit register, then writes data from an 8-bit register.
    /// This is a common pattern for memory-indirect store operations like LD (HL),r8.
    fn WriteBusReg(comptime addr: []const u8, comptime reg: anytype) type {
        return Join(.{ AssertAddrReg(addr), WriteBus(reg) });
    }

    /// Instruction fetch microcode generator.
    /// Combines PC increment with bus read to fetch the next byte from memory.
    /// This is the standard way Game Boy DMG fetches instruction bytes and immediate data.
    fn Fetch(comptime reg: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                switch (stage) {
                    0 => {
                        pins.addr = state.pc.w;
                        state.pc.w += 1; // Increment PC for next fetch
                    },
                    else => {},
                }
                ReadBus(reg).op(ir, stage, state, pins);
            }
        };
    }

    /// Unified register target resolution function.
    /// Accepts either a bit position (for instruction-encoded regs) or a string (for direct regs).
    /// Examples: target(3, ir, state) extracts register from bits 5-3 of instruction
    ///          target("z.l", ir, state) directly references the z.l register
    fn r8(comptime reg: anytype, comptime ir: u8, state: *State) *u8 {
        return switch (@TypeOf(reg)) {
            // Register encoded in instruction - extract bits and map to register
            comptime_int, u3 => state.idx(switch (extract(ir, reg, 3)) {
                0 => "bc.h", // B register
                1 => "bc.l", // C register
                2 => "de.h", // D register
                3 => "de.l", // E register
                4 => "hl.h", // H register
                5 => "hl.l", // L register
                6, 7 => "af.h", // (HL) or A register
            }),
            // Direct register reference - use string directly
            else => state.idx(reg),
        };
    }

    /// Register transfer microcode generator.
    /// Implements register-to-register moves that complete in a single cycle.
    /// Uses the new TargetReg system for flexible register addressing.
    fn Transfer(comptime dst: anytype, comptime src: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                switch (stage) {
                    // Transfer happens immediately in cycle 0
                    0 => r8(dst, ir, state).* = r8(src, ir, state).*,
                    else => {},
                }
            }
        };
    }

    /// Combine multiple microcode operations into a single machine cycle.
    /// Allows multiple simple operations to execute simultaneously.
    fn Join(comptime ops: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                inline for (ops) |code| {
                    code.op(ir, stage, state, pins);
                }
            }
        };
    }

    // === Stub microcode functions for unimplemented operations ===
    // These need proper implementation - currently act as NOPs

    fn Transfer16(comptime dst: anytype, comptime src: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 16-bit register transfer
                    _ = ir;
                    _ = state;
                    _ = dst;
                    _ = src;
                }
            }
        };
    }

    fn SetHighAddr(comptime offset_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                switch (stage) {
                    0 => pins.*.addr = 0xFF00 + r8(offset_reg, ir, state).*,
                    else => {},
                }
            }
        };
    }

    fn IncDec8(comptime reg_pos: anytype, comptime is_inc: bool) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 8-bit increment/decrement with flags
                    _ = ir;
                    _ = state;
                    _ = reg_pos;
                    _ = is_inc;
                }
            }
        };
    }

    fn IncDec8Direct(comptime reg: []const u8, comptime is_inc: bool) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement direct 8-bit increment/decrement
                    _ = ir;
                    _ = state;
                    _ = reg;
                    _ = is_inc;
                }
            }
        };
    }

    fn IncDec16(comptime reg_pos: anytype, comptime is_inc: bool) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 16-bit increment/decrement
                    _ = ir;
                    _ = state;
                    _ = reg_pos;
                    _ = is_inc;
                }
            }
        };
    }

    fn Add8(comptime src_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 8-bit addition with flags
                    _ = ir;
                    _ = state;
                    _ = src_pos;
                }
            }
        };
    }

    fn Add8Direct(comptime src_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement direct 8-bit addition
                    _ = ir;
                    _ = state;
                    _ = src_reg;
                }
            }
        };
    }

    fn Adc8(comptime src_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 8-bit addition with carry
                    _ = ir;
                    _ = state;
                    _ = src_pos;
                }
            }
        };
    }

    fn Adc8Direct(comptime src_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement direct 8-bit addition with carry
                    _ = ir;
                    _ = state;
                    _ = src_reg;
                }
            }
        };
    }

    fn Sub8(comptime src_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 8-bit subtraction with flags
                    _ = ir;
                    _ = state;
                    _ = src_pos;
                }
            }
        };
    }

    fn Sub8Direct(comptime src_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement direct 8-bit subtraction
                    _ = ir;
                    _ = state;
                    _ = src_reg;
                }
            }
        };
    }

    fn Sbc8(comptime src_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 8-bit subtraction with carry
                    _ = ir;
                    _ = state;
                    _ = src_pos;
                }
            }
        };
    }

    fn Sbc8Direct(comptime src_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement direct 8-bit subtraction with carry
                    _ = ir;
                    _ = state;
                    _ = src_reg;
                }
            }
        };
    }

    fn And8(comptime src_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 8-bit logical AND
                    _ = ir;
                    _ = state;
                    _ = src_pos;
                }
            }
        };
    }

    fn And8Direct(comptime src_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement direct 8-bit logical AND
                    _ = ir;
                    _ = state;
                    _ = src_reg;
                }
            }
        };
    }

    fn Xor8(comptime src_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 8-bit logical XOR
                    _ = ir;
                    _ = state;
                    _ = src_pos;
                }
            }
        };
    }

    fn Xor8Direct(comptime src_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement direct 8-bit logical XOR
                    _ = ir;
                    _ = state;
                    _ = src_reg;
                }
            }
        };
    }

    fn Or8(comptime src_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 8-bit logical OR
                    _ = ir;
                    _ = state;
                    _ = src_pos;
                }
            }
        };
    }

    fn Or8Direct(comptime src_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement direct 8-bit logical OR
                    _ = ir;
                    _ = state;
                    _ = src_reg;
                }
            }
        };
    }

    fn Cp8(comptime src_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement 8-bit compare (subtract without storing result)
                    _ = ir;
                    _ = state;
                    _ = src_pos;
                }
            }
        };
    }

    fn Cp8Direct(comptime src_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Implement direct 8-bit compare
                    _ = ir;
                    _ = state;
                    _ = src_reg;
                }
            }
        };
    }

    fn SetPC(comptime addr_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Set PC to address in register
                    _ = ir;
                    _ = state;
                    _ = addr_reg;
                }
            }
        };
    }

    fn NoIrOp() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                // This is used with NoIr to prevent auto-fetching next instruction
                _ = ir;
                _ = stage;
                _ = state;
            }
        };
    }

    fn JumpCond(comptime cc_pos: anytype, comptime addr_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Jump if condition is met, otherwise continue to next instruction
                    _ = ir;
                    _ = state;
                    _ = cc_pos;
                    _ = addr_reg;
                }
            }
        };
    }

    fn JumpRel(comptime offset_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Add signed offset to PC
                    _ = ir;
                    _ = state;
                    _ = offset_reg;
                }
            }
        };
    }

    fn JumpRelCond(comptime cc_pos: anytype, comptime offset_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Jump relative if condition is met
                    _ = ir;
                    _ = state;
                    _ = cc_pos;
                    _ = offset_reg;
                }
            }
        };
    }

    fn DecSP() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Decrement stack pointer
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn IncSP() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Increment stack pointer
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn GetPushReg(comptime rr_pos: anytype, comptime is_high: bool) []const u8 {
        // TODO: Return correct register name for push operations
        _ = rr_pos;
        _ = is_high;
        return "z.l"; // Placeholder
    }

    fn GetPopReg(comptime rr_pos: anytype, comptime is_high: bool) []const u8 {
        // TODO: Return correct register name for pop operations
        _ = rr_pos;
        _ = is_high;
        return "z.l"; // Placeholder
    }

    fn CallCond(comptime cc_pos: anytype, comptime addr_reg: []const u8) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Call if condition is met
                    _ = ir;
                    _ = state;
                    _ = cc_pos;
                    _ = addr_reg;
                }
            }
        };
    }

    fn RetCond(comptime cc_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Return if condition is met
                    _ = ir;
                    _ = state;
                    _ = cc_pos;
                }
            }
        };
    }

    fn EnableInterrupts() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Enable interrupts
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn DisableInterrupts() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Disable interrupts
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn RestartJump(comptime rst_pos: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Jump to restart vector (rst_pos * 8)
                    _ = ir;
                    _ = state;
                    _ = rst_pos;
                }
            }
        };
    }

    fn HaltCPU() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Halt CPU until interrupt
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn StopCPU() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Stop CPU and LCD
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn ComplementCarry() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Complement carry flag
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn SetCarry() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Set carry flag
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn ComplementA() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Complement A register (bitwise NOT)
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn DecimalAdjust() type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Decimal adjust A for BCD arithmetic
                    _ = ir;
                    _ = state;
                }
            }
        };
    }

    fn RotateLeftA(comptime through_carry: bool) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Rotate A left (circular or through carry)
                    _ = ir;
                    _ = state;
                    _ = through_carry;
                }
            }
        };
    }

    fn RotateRightA(comptime through_carry: bool) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    // TODO: Rotate A right (circular or through carry)
                    _ = ir;
                    _ = state;
                    _ = through_carry;
                }
            }
        };
    }

    /// Complete Game Boy DMG instruction set decoder with microcode implementations.
    /// This defines the actual instruction set architecture of the emulated CPU.
    /// Game Boy DMG instruction set implementation.
    /// Each .add() call defines an instruction with its bit pattern and microcode sequence.
    /// Patterns use: '0'/'1' for fixed bits, 'x' for variable bits.
    const decoder = Decoder.init
        // === 8-bit Load Instructions ===
        // NOP - just fetch next instruction
        .add("00000000", .{Fetch("ir")})
        // LD r8,r8 - copy register to register
        .add("01xxxxxx", .{Join(.{ Fetch("ir"), Transfer(3, 0) })})
        // LD r8,n - load immediate 8-bit value
        .add("00xxx110", .{
            Fetch("z.l"), // Fetch immediate byte into temp
            Join(.{ Fetch("ir"), Transfer(3, "z.l") }), // Move temp to dest, fetch next instruction
        })
        // LD r8,(HL) - load from memory address in HL register
        .add("01xxx110", .{
            ReadBusReg("hl", "z.l"), // Read byte from [HL] into temp
            Join(.{ Fetch("ir"), Transfer(3, "z.l") }), // Move temp to dest, fetch next instruction
        })
        // LD (HL),r8 - store to memory address in HL register
        .add("01110xxx", .{
            WriteBusReg("hl", 0), // Write r8 at bit index 0 to [HL]
            Fetch("ir"), // Fetch next instruction
        })
        // LD (HL),n - store immediate to memory address in HL
        .add("00110110", .{
            Fetch("z.l"), // Fetch immediate byte
            WriteBusReg("hl", "z.l"), // Write immediate to [HL]
            Fetch("ir"), // Fetch next instruction
        })

        // === 16-bit Load Instructions ===
        // LD rr,nn - load 16-bit immediate into register pair
        .add("00xx0001", .{
            Fetch("z.l"), // Fetch low byte
            Fetch("z.h"), // Fetch high byte
            Join(.{ Fetch("ir"), Transfer16(4, "z") }), // Move to destination register pair
        })
        // LD SP,HL - copy HL to SP
        .add("11111001", .{Join(.{ Fetch("ir"), Transfer16("sp", "hl") })})

        // === Memory Load Instructions ===
        // LD A,(BC) - load from address in BC
        .add("00001010", .{
            ReadBusReg("bc", "af.h"), // Read from [BC] into A
            Fetch("ir"), // Fetch next instruction
        })
        // LD A,(DE) - load from address in DE
        .add("00011010", .{
            ReadBusReg("de", "af.h"), // Read from [DE] into A
            Fetch("ir"), // Fetch next instruction
        })
        // LD (BC),A - store A to address in BC
        .add("00000010", .{
            WriteBusReg("bc", "af.h"), // Write A to [BC]
            Fetch("ir"), // Fetch next instruction
        })
        // LD (DE),A - store A to address in DE
        .add("00010010", .{
            WriteBusReg("de", "af.h"), // Write A to [DE]
            Fetch("ir"), // Fetch next instruction
        })
        // LD A,(nn) - load from 16-bit address
        .add("11111010", .{
            Fetch("z.l"), // Fetch address low byte
            Fetch("z.h"), // Fetch address high byte
            ReadBusReg("z", "af.h"), // Read from [nn] into A
            Fetch("ir"), // Fetch next instruction
        })
        // LD (nn),A - store A to 16-bit address
        .add("11101010", .{
            Fetch("z.l"), // Fetch address low byte
            Fetch("z.h"), // Fetch address high byte
            WriteBusReg("z", "af.h"), // Write A to [nn]
            Fetch("ir"), // Fetch next instruction
        })

        // === High Memory Instructions ===
        // LDH A,(n) - load from high memory (0xFF00+n)
        .add("11110000", .{
            Fetch("z.l"), // Fetch offset
            Join(.{ SetHighAddr("z.l"), ReadBus("af.h") }), // Read from 0xFF00+n into A
            Fetch("ir"), // Fetch next instruction
        })
        // LDH (n),A - store to high memory (0xFF00+n)
        .add("11100000", .{
            Fetch("z.l"), // Fetch offset
            Join(.{ SetHighAddr("z.l"), WriteBus("af.h") }), // Write A to 0xFF00+n
            Fetch("ir"), // Fetch next instruction
        })
        // LDH A,(C) - load from high memory (0xFF00+C)
        .add("11110010", .{
            Join(.{ SetHighAddr("bc.l"), ReadBus("af.h") }), // Read from 0xFF00+C into A
            Fetch("ir"), // Fetch next instruction
        })
        // LDH (C),A - store to high memory (0xFF00+C)
        .add("11100010", .{
            Join(.{ SetHighAddr("bc.l"), WriteBus("af.h") }), // Write A to 0xFF00+C
            Fetch("ir"), // Fetch next instruction
        })

        // === Increment/Decrement Instructions ===
        // INC r8 - increment 8-bit register
        .add("00xxx100", .{Join(.{ Fetch("ir"), IncDec8(3, true) })})
        // DEC r8 - decrement 8-bit register
        .add("00xxx101", .{Join(.{ Fetch("ir"), IncDec8(3, false) })})
        // INC (HL) - increment memory at HL
        .add("00110100", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ IncDec8Direct("z.l", true), WriteBusReg("hl", "z.l") }), // Inc and write back
            Fetch("ir"), // Fetch next instruction
        })
        // DEC (HL) - decrement memory at HL
        .add("00110101", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ IncDec8Direct("z.l", false), WriteBusReg("hl", "z.l") }), // Dec and write back
            Fetch("ir"), // Fetch next instruction
        })

        // === 16-bit Arithmetic ===
        // INC rr - increment 16-bit register
        .add("00xx0011", .{Join(.{ Fetch("ir"), IncDec16(4, true) })})
        // DEC rr - decrement 16-bit register
        .add("00xx1011", .{Join(.{ Fetch("ir"), IncDec16(4, false) })})

        // === Arithmetic Instructions ===
        // ADD A,r8 - add register to A
        .add("10000xxx", .{Join(.{ Fetch("ir"), Add8(0) })})
        // ADD A,n - add immediate to A
        .add("11000110", .{
            Fetch("z.l"), // Fetch immediate
            Join(.{ Fetch("ir"), Add8Direct("z.l") }), // Add immediate to A
        })
        // ADD A,(HL) - add memory to A
        .add("10000110", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ Fetch("ir"), Add8Direct("z.l") }), // Add to A
        })
        // ADC A,r8 - add with carry
        .add("10001xxx", .{Join(.{ Fetch("ir"), Adc8(0) })})
        // ADC A,n - add immediate with carry
        .add("11001110", .{
            Fetch("z.l"), // Fetch immediate
            Join(.{ Fetch("ir"), Adc8Direct("z.l") }), // Add with carry to A
        })
        // ADC A,(HL) - add memory with carry
        .add("10001110", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ Fetch("ir"), Adc8Direct("z.l") }), // Add with carry to A
        })

        // === Subtraction Instructions ===
        // SUB r8 - subtract register from A
        .add("10010xxx", .{Join(.{ Fetch("ir"), Sub8(0) })})
        // SUB n - subtract immediate from A
        .add("11010110", .{
            Fetch("z.l"), // Fetch immediate
            Join(.{ Fetch("ir"), Sub8Direct("z.l") }), // Subtract from A
        })
        // SUB (HL) - subtract memory from A
        .add("10010110", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ Fetch("ir"), Sub8Direct("z.l") }), // Subtract from A
        })
        // SBC A,r8 - subtract with carry
        .add("10011xxx", .{Join(.{ Fetch("ir"), Sbc8(0) })})
        // SBC A,n - subtract immediate with carry
        .add("11011110", .{
            Fetch("z.l"), // Fetch immediate
            Join(.{ Fetch("ir"), Sbc8Direct("z.l") }), // Subtract with carry from A
        })
        // SBC A,(HL) - subtract memory with carry
        .add("10011110", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ Fetch("ir"), Sbc8Direct("z.l") }), // Subtract with carry from A
        })

        // === Logic Instructions ===
        // AND r8 - logical AND with A
        .add("10100xxx", .{Join(.{ Fetch("ir"), And8(0) })})
        // AND n - logical AND immediate with A
        .add("11100110", .{
            Fetch("z.l"), // Fetch immediate
            Join(.{ Fetch("ir"), And8Direct("z.l") }), // AND with A
        })
        // AND (HL) - logical AND memory with A
        .add("10100110", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ Fetch("ir"), And8Direct("z.l") }), // AND with A
        })
        // XOR r8 - logical XOR with A
        .add("10101xxx", .{Join(.{ Fetch("ir"), Xor8(0) })})
        // XOR n - logical XOR immediate with A
        .add("11101110", .{
            Fetch("z.l"), // Fetch immediate
            Join(.{ Fetch("ir"), Xor8Direct("z.l") }), // XOR with A
        })
        // XOR (HL) - logical XOR memory with A
        .add("10101110", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ Fetch("ir"), Xor8Direct("z.l") }), // XOR with A
        })
        // OR r8 - logical OR with A
        .add("10110xxx", .{Join(.{ Fetch("ir"), Or8(0) })})
        // OR n - logical OR immediate with A
        .add("11110110", .{
            Fetch("z.l"), // Fetch immediate
            Join(.{ Fetch("ir"), Or8Direct("z.l") }), // OR with A
        })
        // OR (HL) - logical OR memory with A
        .add("10110110", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ Fetch("ir"), Or8Direct("z.l") }), // OR with A
        })
        // CP r8 - compare register with A
        .add("10111xxx", .{Join(.{ Fetch("ir"), Cp8(0) })})
        // CP n - compare immediate with A
        .add("11111110", .{
            Fetch("z.l"), // Fetch immediate
            Join(.{ Fetch("ir"), Cp8Direct("z.l") }), // Compare with A
        })
        // CP (HL) - compare memory with A
        .add("10111110", .{
            ReadBusReg("hl", "z.l"), // Read from [HL]
            Join(.{ Fetch("ir"), Cp8Direct("z.l") }), // Compare with A
        })

        // === Jump Instructions ===
        // JP nn - absolute jump
        .add("11000011", .{
            Fetch("z.l"), // Fetch address low
            Fetch("z.h"), // Fetch address high
            Join(.{ SetPC("z"), NoIrOp() }), // Set PC and don't auto-fetch
        })
        // JP cc,nn - conditional jump
        .add("110xx010", .{
            Fetch("z.l"), // Fetch address low
            Fetch("z.h"), // Fetch address high
            JumpCond(3, "z"), // Jump if condition met
        })
        // JP (HL) - jump to address in HL
        .add("11101001", .{Join(.{ SetPC("hl"), NoIrOp() })})
        // JR n - relative jump
        .add("00011000", .{
            Fetch("z.l"), // Fetch offset
            Join(.{ JumpRel("z.l"), NoIrOp() }), // Jump relative
        })
        // JR cc,n - conditional relative jump
        .add("001xx000", .{
            Fetch("z.l"), // Fetch offset
            JumpRelCond(3, "z.l"), // Jump relative if condition met
        })

        // === Stack Instructions ===
        // PUSH rr - push register pair to stack
        .add("11xx0101", .{
            Join(.{ DecSP(), WriteBusReg("sp", GetPushReg(4, true)) }), // Push high byte
            Join(.{ DecSP(), WriteBusReg("sp", GetPushReg(4, false)) }), // Push low byte
            Fetch("ir"), // Fetch next instruction
        })
        // POP rr - pop register pair from stack
        .add("11xx0001", .{
            ReadBusReg("sp", GetPopReg(4, false)), // Pop low byte
            Join(.{ IncSP(), ReadBusReg("sp", GetPopReg(4, true)) }), // Pop high byte
            Join(.{ IncSP(), Fetch("ir") }), // Increment SP and fetch next
        })

        // === Call/Return Instructions ===
        // CALL nn - call subroutine
        .add("11001101", .{
            Fetch("z.l"), // Fetch address low
            Fetch("z.h"), // Fetch address high
            Join(.{ DecSP(), WriteBusReg("sp", "pc.h") }), // Push PC high
            Join(.{ DecSP(), WriteBusReg("sp", "pc.l") }), // Push PC low
            Join(.{ SetPC("z"), NoIrOp() }), // Jump to address
        })
        // CALL cc,nn - conditional call
        .add("110xx100", .{
            Fetch("z.l"), // Fetch address low
            Fetch("z.h"), // Fetch address high
            CallCond(3, "z"), // Call if condition met
        })
        // RET - return from subroutine
        .add("11001001", .{
            ReadBusReg("sp", "pc.l"), // Pop PC low
            Join(.{ IncSP(), ReadBusReg("sp", "pc.h") }), // Pop PC high
            Join(.{ IncSP(), NoIrOp() }), // Increment SP and don't auto-fetch
        })
        // RET cc - conditional return
        .add("110xx000", .{RetCond(3)})
        // RETI - return and enable interrupts
        .add("11011001", .{
            ReadBusReg("sp", "pc.l"), // Pop PC low
            Join(.{ IncSP(), ReadBusReg("sp", "pc.h") }), // Pop PC high
            Join(.{ IncSP(), EnableInterrupts(), NoIrOp() }), // Enable interrupts and don't auto-fetch
        })

        // === Restart Instructions ===
        // RST n - restart (call to fixed address)
        .add("11xxx111", .{
            Join(.{ DecSP(), WriteBusReg("sp", "pc.h") }), // Push PC high
            Join(.{ DecSP(), WriteBusReg("sp", "pc.l") }), // Push PC low
            Join(.{ RestartJump(3), NoIrOp() }), // Jump to restart vector
        })

        // === Miscellaneous Instructions ===
        // HALT - halt until interrupt
        .add("01110110", .{Join(.{ HaltCPU(), Fetch("ir") })})
        // STOP - stop CPU and LCD
        .add("00010000", .{
            Fetch("z.l"), // Fetch next byte (should be 0x00)
            Join(.{ StopCPU(), Fetch("ir") }), // Stop CPU
        })
        // DI - disable interrupts
        .add("11110011", .{Join(.{ DisableInterrupts(), Fetch("ir") })})
        // EI - enable interrupts
        .add("11111011", .{Join(.{ EnableInterrupts(), Fetch("ir") })})
        // CCF - complement carry flag
        .add("00111111", .{Join(.{ ComplementCarry(), Fetch("ir") })})
        // SCF - set carry flag
        .add("00110111", .{Join(.{ SetCarry(), Fetch("ir") })})
        // CPL - complement A register
        .add("00101111", .{Join(.{ ComplementA(), Fetch("ir") })})
        // DAA - decimal adjust A
        .add("00100111", .{Join(.{ DecimalAdjust(), Fetch("ir") })})

        // === Rotate/Shift Instructions ===
        // RLCA - rotate A left circular
        .add("00000111", .{Join(.{ RotateLeftA(false), Fetch("ir") })})
        // RLA - rotate A left through carry
        .add("00010111", .{Join(.{ RotateLeftA(true), Fetch("ir") })})
        // RRCA - rotate A right circular
        .add("00001111", .{Join(.{ RotateRightA(false), Fetch("ir") })})
        // RRA - rotate A right through carry
        .add("00011111", .{Join(.{ RotateRightA(true), Fetch("ir") })})
        .Build(null);
};

/// Execute one CPU clock cycle.
/// This is the main entry point that advances the CPU state by one tick.
pub inline fn tick(state: *State, pins: *Pins) void {
    state.exec.stage(state, pins);
}

/// Test harness for CPU emulation.
/// Provides a simulated Game Boy environment with ROM and RAM for testing instructions.
const Tester = struct {
    /// CPU internal state
    state: State,
    /// CPU external pins
    pins: Pins,
    /// Simulated cartridge ROM (0x0000-0x7FFF)
    rom: [0x8000]u8,
    /// Simulated work RAM (0xC000-0xDFFF)
    ram: [0x2000]u8,

    /// Initialize test environment with ROM data
    fn init(rom: []const u8) @This() {
        var this = @This(){
            .state = .reset,
            .pins = .zero,
            .rom = [1]u8{0} ** 0x8000,
            .ram = [1]u8{0} ** 0x2000,
        };
        @memcpy(this.rom[0..rom.len], rom[0..]);
        return this;
    }

    /// Run CPU simulation for specified number of ticks.
    /// Includes bus simulation to respond to memory accesses.
    /// Note: +4 ticks accounts for initial NOP instruction fetch.
    fn run(this: *@This(), ticks: usize, verbose: bool) void {
        var n: usize = 0;
        while (n < ticks + 4) : (n += 1) {
            tick(&this.state, &this.pins);

            if (verbose) {
                std.debug.print("tick: {}\nregs: \n", .{n});
                inline for (comptime std.meta.fieldNames(State)) |name| {
                    switch (@FieldType(State, name)) {
                        State.Reg(16) => std.debug.print(
                            "\t{s}: {X:0>4}\n",
                            .{ name, @field(this.*.state, name).w },
                        ),
                        State.Reg(8) => std.debug.print(
                            "\t{s}: {X:0>2}\n",
                            .{ name, @field(this.*.state, name).b },
                        ),
                        else => {},
                    }
                }
                std.debug.print("\n", .{});
            }

            // Simulate Game Boy memory system responses
            if (this.pins.addr < 0x8000) {
                // Cartridge ROM area (0x0000-0x7FFF)
                if (this.pins.rd) {
                    this.pins.data = this.rom[this.pins.addr];
                }
            } else if (this.pins.addr >= 0xC000 and this.pins.addr < 0xDFFF) {
                // Work RAM area (0xC000-0xDFFF)
                if (this.pins.rd) {
                    this.pins.data = this.ram[this.pins.addr - 0xC000];
                } else if (this.pins.wr) {
                    this.ram[this.pins.addr - 0xC000] = this.pins.data;
                }
            }

            if (verbose) {
                std.debug.print("pins: {}\n", .{this.*.pins});
            }
        }
    }
};

// Test cases for verifying CPU instruction implementation

test "nop" {
    var tester = Tester.init(&.{ 0x00, 0x42 });
    tester.run(4, false);
    try std.testing.expectEqual(2, tester.state.pc.w); // PC should advance by 2
    try std.testing.expectEqual(0x42, tester.state.ir.b); // Should have fetched 0x42
}

test "ld r8, r8" {
    var tester = Tester.init(&.{0b01_000_001}); // LD B,C
    tester.state.bc.b.l = 0x42; // Set C = 0x42
    tester.run(4, false);
    try std.testing.expectEqual(0x42, tester.state.bc.b.h); // B should now equal C
}

test "ld r8, n" {
    var tester = Tester.init(&.{ 0b00_111_110, 0x42 }); // LD A,0x42
    tester.run(8, false);
    try std.testing.expectEqual(0x42, tester.state.af.b.h); // A should equal 0x42
}

test "ld r8, (hl)" {
    var tester = Tester.init(&.{0b01_111_110}); // LD A,(HL)
    tester.state.hl.w = 0xC000;
    tester.ram[0] = 0x42;
    tester.run(8, false);
    try std.testing.expectEqual(0x42, tester.state.af.b.h); // A should equal 0x42
}

test "ld (hl), r8" {
    var tester = Tester.init(&.{0b01_110_111}); // LD (HL),A
    tester.state.hl.w = 0xC000;
    tester.state.af.b.h = 0x42;
    tester.run(8, false);
    try std.testing.expectEqual(0x42, tester.ram[0]); // [0xC000] should equal 0x42
}
