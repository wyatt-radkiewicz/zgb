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
pub const Pins = packed struct {
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

    /// Specifies which half of a 16-bit register to access
    const Half = enum { l, h };

    /// Compile-time generated enum for register indexing.
    /// This creates a type-safe way to reference registers without using strings.
    /// The compiler generates this by introspecting the State struct fields.
    const Index = create_register_index: {
        // Collect all register field names from the State struct
        var variants = std.BoundedArray([:0]const u8, 64).init(0) catch unreachable;
        for (std.meta.fields(State)) |field| {
            switch (field.type) {
                Reg(8), Reg(16) => variants.appendAssumeCapacity(field.name),
                else => {}, // Skip non-register fields
            }
        }

        // Generate a tagged union type with one variant per register
        break :create_register_index @Type(.{
            .@"union" = .{
                .layout = .auto,

                // Create enum tags for each register
                .tag_type = @Type(.{ .@"enum" = .{
                    .tag_type = std.math.IntFittingRange(0, variants.len),
                    .fields = tag_fields: {
                        var tags: [variants.len]std.builtin.Type.EnumField = undefined;
                        for (variants.slice(), 0..) |variant, i| {
                            tags[i] = .{ .name = variant, .value = i };
                        }
                        break :tag_fields &tags;
                    },
                    .decls = &.{},
                    .is_exhaustive = false,
                } }),

                // Create union fields - 16-bit regs need half specifier, 8-bit don't
                .fields = union_fields: {
                    var fields: [variants.len]std.builtin.Type.UnionField = undefined;
                    for (variants.slice(), &fields) |variant, *field| {
                        field.* = .{
                            .name = variant,
                            .type = switch (@FieldType(State, variant)) {
                                Reg(8) => void, // 8-bit: no half needed
                                Reg(16) => Half, // 16-bit: specify high or low
                                else => unreachable,
                            },
                            .alignment = 0,
                        };
                    }
                    break :union_fields &fields;
                },

                .decls = &.{},
            },
        });
    };

    /// Get a pointer to a specific register byte for reading/writing.
    /// This handles the complexity of accessing different register formats uniformly.
    inline fn idx(this: *@This(), comptime index: Index) *u8 {
        const name = @tagName(index);
        return switch (index) {
            inline else => |i| switch (@TypeOf(i)) {
                void => &@field(this.*, name).b, // 8-bit register
                else => &@field(@field(this.*, name).b, @tagName(i)), // 16-bit register half
            },
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
        fn init(comptime enc: *const [8]u8, comptime code: []const type) @This() {
            @setEvalBranchQuota(1000000);
            var ones: u8 = 0; // Required '1' bits
            var care: u8 = 0; // Bits we care about (not 'x')

            // Parse the encoding string
            for (enc, 0..) |c, i| {
                switch (c) {
                    '0', '1', 'x' => {},
                    else => @compileError(std.fmt.comptimePrint(
                        "Expected '0', '1' or 'x' in encoding string but found \"{}\"",
                        .{enc},
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
    fn instr(comptime this: @This(), comptime enc: *const [8]u8, comptime code: anytype) @This() {
        const codes = @as([code.len]type, code);
        const encoding = Encoding.init(enc, &codes);

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
    fn Build(comptime this: @This()) type {
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
            fn link(comptime enc: Encoding) [256]*const Microcode {
                const noir = enc.code[enc.code.len - 1] == NoIr;
                comptime var state = enc.code.len - @intFromBool(noir);
                comptime var last = [1]?*const Microcode{null} ** 256;

                // Process each microcode stage (working backwards for linking)
                inline while (state != 0) {
                    state -= 1;
                    comptime var stage = 4;

                    // Each stage has 4 clock cycles
                    inline while (stage != 0) {
                        stage -= 1;
                        inline for (0..last.len) |byte| {
                            if (!enc.mask.isSet(byte)) {
                                continue; // Skip opcodes this encoding doesn't handle
                            }

                            // Create microcode function for this stage
                            last[byte] = struct {
                                fn op(s: *State, p: *Pins) void {
                                    // Execute this stage
                                    enc.code[state].op(byte, stage, s, p);

                                    // Link to next stage or set IR-based microcode
                                    if (last[byte]) |pfn| {
                                        s.exec.stage = pfn; // Continue to next stage
                                    } else if (!noir) {
                                        // Auto-set microcode from IR lookup
                                        s.exec.stage = lut[s.ir.b];
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

    /// Convert 3-bit register encoding to register index.
    /// This maps the r8 addressing mode used in many Game Boy DMG instructions:
    /// 000=B, 001=C, 010=D, 011=E, 100=H, 101=L, 110=(HL), 111=A
    fn r8(enc: u3) State.Index {
        return switch (enc) {
            0 => .{ .bc = .h }, // B register
            1 => .{ .bc = .l }, // C register
            2 => .{ .de = .h }, // D register
            3 => .{ .de = .l }, // E register
            4 => .{ .hl = .h }, // H register
            5 => .{ .hl = .l }, // L register
            6, 7 => .{ .af = .h }, // (HL) or A register
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
    fn ReadBus(comptime reg: State.Index) type {
        return struct {
            pub fn op(comptime _: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                switch (stage) {
                    0 => {
                        pins.*.rd = true;
                        pins.*.wr = false;
                        pins.*.cs = true;
                    },
                    1 => pins.*.cs = cs(pins.*.addr),
                    2 => {}, // Wait state for memory access
                    3 => state.idx(reg).* = pins.*.data,
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
    fn WriteBus(comptime reg: State.Index) type {
        return struct {
            pub fn op(comptime _: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                switch (stage) {
                    0 => {
                        pins.*.rd = true;
                        pins.*.wr = false;
                        pins.*.cs = true;
                    },
                    1 => pins.*.cs = cs(pins.*.addr),
                    2 => {
                        pins.*.wr = true;
                        pins.*.data = state.idx(reg).*;
                    },
                    3 => {}, // Complete write cycle
                }
            }
        };
    }

    /// Instruction fetch microcode generator.
    /// Combines PC increment with bus read to fetch the next byte from memory.
    /// This is the standard way Game Boy DMG fetches instruction bytes and immediate data.
    fn Fetch(comptime reg: State.Index) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                if (stage == 0) {
                    pins.addr = state.pc.w;
                    state.pc.w += 1; // Increment PC for next fetch
                }
                ReadBus(reg).op(ir, stage, state, pins);
            }
        };
    }

    /// Register transfer microcode generator.
    /// Implements register-to-register moves that complete in a single cycle.
    /// Supports both direct register references and instruction-encoded registers.
    fn Transfer(comptime dst: u3, comptime src: union(enum) { ir: u3, reg: State.Index }) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) { // Transfer happens immediately in cycle 0
                    const from = switch (src) {
                        // Register encoded in instruction
                        .ir => |idx| state.idx(r8(extract(ir, idx, 3))).*,
                        // Direct register reference
                        .reg => |reg| state.idx(reg).*,
                    };
                    state.idx(r8(extract(ir, dst, 3))).* = from;
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

    /// Complete Game Boy DMG instruction set decoder with microcode implementations.
    /// This defines the actual instruction set architecture of the emulated CPU.
    const decoder = Decoder.init
        .instr("00000000", .{ // NOP - just fetch next instruction
            Fetch(.ir),
        })
        .instr("01xxxxxx", .{ // LD r8,r8 - register to register
            Join(.{ Fetch(.ir), Transfer(3, .{ .ir = 0 }) }),
        })
        .instr("00xxx110", .{ // LD r8,n - load immediate byte
            // Fetch immediate byte into temp register
            Fetch(.{ .z = .l }),
            // Transfer to destination and fetch next
            Join(.{ Fetch(.ir), Transfer(3, .{ .reg = .{ .z = .l } }) }),
        })
        .Build();
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
    fn run(this: *@This(), ticks: usize) void {
        var n: usize = 0;
        while (n < ticks + 4) : (n += 1) {
            tick(&this.state, &this.pins);

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
        }
    }
};

// Test cases for verifying CPU instruction implementation

test "nop" {
    var tester = Tester.init(&.{ 0x00, 0x42 });
    tester.run(4);
    try std.testing.expectEqual(2, tester.state.pc.w); // PC should advance by 2
    try std.testing.expectEqual(0x42, tester.state.ir.b); // Should have fetched 0x42
}

test "ld r8, r8" {
    var tester = Tester.init(&.{0b01_000_001}); // LD B,C
    tester.state.bc.b.l = 0x42; // Set C = 0x42
    tester.run(4);
    try std.testing.expectEqual(0x42, tester.state.bc.b.h); // B should now equal C
}

test "ld r8, n" {
    var tester = Tester.init(&.{ 0b00_111_110, 0x42 }); // LD A,0x42
    tester.run(8);
    try std.testing.expectEqual(0x42, tester.state.af.b.h); // A should equal 0x42
}
