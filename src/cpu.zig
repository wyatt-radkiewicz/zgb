const std = @import("std");

/// Represents the physical pins on the chip
pub const Pins = packed struct {
    /// Address lines (output only)
    addr: u16,

    /// Data lines (input/ouput)
    data: u8,

    /// Chip select pin
    cs: bool,

    /// Reading from data lines?
    rd: bool,

    /// Writing to data lines?
    wr: bool,

    pub const zero = @This(){
        .addr = 0,
        .data = 0,
        .cs = false,
        .rd = false,
        .wr = false,
    };
};

/// Execution engine of the cpu, also holds snapshots of internal state like registers
pub const State = struct {
    /// Accumulator and flags register
    af: Reg(16),

    /// General purpose 16/8 bit registers
    bc: Reg(16),
    de: Reg(16),
    hl: Reg(16),

    /// Stack and program counter
    pc: Reg(16),
    sp: Reg(16),

    /// Interrupt enable register
    ie: Reg(8),

    /// Current instruction opcode being processed
    ir: Reg(8),

    /// Temporary buffer for memory transfers
    z: Reg(16),

    /// Execution engine (internal)
    exec: Exec,

    /// State of things after reset
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

    /// Returns the type for a register of the given width
    pub fn Reg(comptime width: u16) type {
        return switch (width) {
            8 => packed struct {
                // Byte
                b: u8,

                pub const zero = @This(){ .b = 0 };
            },
            16 => packed union {
                // Byte
                b: packed struct {
                    // Lower 8 bits (lsb)
                    l: u8,

                    // Higher 8 bits (msb)
                    h: u8,
                },

                // For flags register
                f: packed struct {
                    pad: u4,

                    /// Carry flag
                    c: bool,

                    /// Half Carry flag (BCD)
                    h: bool,

                    /// Subtraction flag (BCD)
                    n: bool,

                    /// Zero flag
                    z: bool,
                },

                // Word, full 16 bits
                w: u16,

                pub const zero = @This(){ .w = 0 };
            },
            else => @compileError(std.fmt.comptimePrint(
                "Expected register width of 8 or 16 but found {}",
                .{width},
            )),
        };
    }

    /// Used to specify which half of a 16 bit register to use for Index
    const Half = enum { l, h };

    /// Used to store a refrence of a register name, not the register itself.
    /// Used in the instruction/microcode code to pass in registers.
    const Index = create_register_index: {
        // Get each field in the state struct that is a register
        var variants = std.BoundedArray([:0]const u8, 64).init(0) catch unreachable;
        for (std.meta.fields(State)) |field| {
            switch (field.type) {
                Reg(8), Reg(16) => variants.appendAssumeCapacity(field.name),
                else => {},
            }
        }

        // The actual index union
        break :create_register_index @Type(.{
            .@"union" = .{
                // Any layout is fine, use auto
                .layout = .auto,

                // Create the tag type
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

                // Create the field.
                // For 16 bit registers, make the type an indeicator of which byte
                // For 8 bit registers, void is fine to represetn the whole
                .fields = union_fields: {
                    var fields: [variants.len]std.builtin.Type.UnionField = undefined;
                    for (variants.slice(), &fields) |variant, *field| {
                        field.* = .{
                            .name = variant,
                            .type = switch (@FieldType(State, variant)) {
                                Reg(8) => void,
                                Reg(16) => Half,
                                else => unreachable,
                            },
                            .alignment = 0,
                        };
                    }
                    break :union_fields &fields;
                },

                // No declarations needed
                .decls = &.{},
            },
        });
    };

    /// Get the address of the variable for setting/getting
    inline fn idx(this: *@This(), comptime index: Index) *u8 {
        const name = @tagName(index);
        return switch (index) {
            inline else => |i| switch (@TypeOf(i)) {
                void => &@field(this.*, name).b,
                else => &@field(@field(this.*, name).b, @tagName(i)),
            },
        };
    }
};

/// Code that simulates 1 clock cycle of the cpu
const Microcode = fn (*State, *Pins) void;

/// Builder pattern used to create a decoder
const Decoder = struct {
    /// All the encoders, ordered from most specific to least specific
    encoders: []const Encoding,

    /// Represents mapping from byte => code.
    const Encoding = struct {
        /// Set bits denote valid bytes for this encoding
        mask: std.bit_set.ArrayBitSet(u64, 256),

        /// Code associated with this encoding
        code: []const type,

        /// Create encoding from encoding string
        /// `0` or `1` denote bits that must be 0 or 1
        /// `x` denotes the bit can be anything
        fn init(comptime enc: *const [8]u8, comptime code: []const type) @This() {
            @setEvalBranchQuota(1000000);
            var ones: u8 = 0;
            var care: u8 = 0;
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
            var mask = std.bit_set.ArrayBitSet(u64, 256).initEmpty();
            inline for (0..mask.capacity()) |i| {
                mask.setValue(i, i ^ ones & care == 0);
            }
            return .{ .mask = mask, .code = code };
        }
    };

    /// Special type used in building the decoder lut to denote not to follow IR at final stage
    const NoIr = struct {};

    /// Initial decoder with only a stub
    const init = @This(){ .encoders = &.{.{
        .mask = .initFull(),
        .code = &.{ struct {
            pub fn op(comptime _: u8, comptime _: u2, _: *State, _: *Pins) void {}
        }, NoIr },
    }} };

    /// Adds an instruction to the encoding, preserving `encodings` order
    fn instr(comptime this: @This(), comptime enc: *const [8]u8, comptime code: anytype) @This() {
        const codes = @as([code.len]type, code);
        const encoding = Encoding.init(enc, &codes);
        const idx = std.sort.lowerBound(Encoding, this.encoders, encoding, struct {
            fn cmp(ctx: Encoding, val: Encoding) std.math.Order {
                return std.math.order(ctx.mask.count(), val.mask.count());
            }
        }.cmp);
        var encoders = this.encoders[0..idx] ++ .{encoding} ++
            this.encoders[idx..];
        return .{ .encoders = encoders[0..] };
    }

    /// Builds the final opcode look up table by returning a type with a decl called `lut`
    /// This also links together all the code in encodings.
    fn Build(comptime this: @This()) type {
        return struct {
            /// The actual look up table present to users
            pub const lut = compute_lut: {
                @setEvalBranchQuota(1000000);

                // Get the code for every encoding
                var code: [this.encoders.len][256]*const Microcode = undefined;
                for (this.encoders, &code) |encoder, *pfn| {
                    pfn.* = link(encoder);
                }

                // Now generate the look up table
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

            /// Links together a encoding's code
            fn link(comptime enc: Encoding) [256]*const Microcode {
                const noir = enc.code[enc.code.len - 1] == NoIr;
                comptime var state = enc.code.len - @intFromBool(noir);
                comptime var last = [1]?*const Microcode{null} ** 256;
                inline while (state != 0) {
                    state -= 1;
                    comptime var stage = 4;
                    inline while (stage != 0) {
                        stage -= 1;
                        inline for (0..last.len) |byte| {
                            if (!enc.mask.isSet(byte)) {
                                continue;
                            }
                            last[byte] = struct {
                                fn op(s: *State, p: *Pins) void {
                                    enc.code[state].op(byte, stage, s, p);
                                    if (last[byte]) |pfn| {
                                        s.exec.stage = pfn;
                                    } else if (!noir) {
                                        s.exec.stage = lut[s.ir.b];
                                    }
                                }
                            }.op;
                        }
                    }
                }
                comptime var final = [1]*const Microcode{struct {
                    pub fn op(_: *State, _: *Pins) void {}
                }.op} ** 256;
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

/// Utilities for the execution engine. Also includes microcode for machine cycles
const Exec = struct {
    /// Microcode stage pointer
    stage: *const Microcode,

    /// Default state for execution engine, set the stage to the microcode for opcode 0x00 (NOP)
    const reset = @This(){ .stage = decoder.lut[0x00] };

    /// Calculates what the cs pin should be based on the address given
    fn cs(addr: u16) bool {
        return switch (addr) {
            0xA000...0xFDFF => false,
            else => true,
        };
    }

    /// Gets the register index from a r8 addressing mode. Only handles the direct modes in r8
    fn r8(enc: u3) State.Index {
        return switch (enc) {
            0 => .{ .bc = .h },
            1 => .{ .bc = .l },
            2 => .{ .de = .h },
            3 => .{ .de = .l },
            4 => .{ .hl = .h },
            5 => .{ .hl = .l },
            6, 7 => .{ .af = .h },
        };
    }

    /// Extracts bits from another integer
    inline fn extract(
        from: anytype,
        idx: std.math.Log2Int(@TypeOf(from)),
        cnt: u16,
    ) std.meta.Int(.unsigned, cnt) {
        return @truncate(from >> idx);
    }

    /// Reads from the bus a value. Set the address pins before starting the read bus operation
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
                    2 => {},
                    3 => state.idx(reg).* = pins.*.data,
                }
            }
        };
    }

    /// Writes to the bus a value. Set the address pins before starting the write bus operation
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
                    3 => {},
                }
            }
        };
    }

    /// Increments the program counter and stores it to the variable
    fn Fetch(comptime reg: State.Index) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                if (stage == 0) {
                    pins.addr = state.pc.w;
                    state.pc.w += 1;
                }
                ReadBus(reg).op(ir, stage, state, pins);
            }
        };
    }

    /// Transfers from 1 8 bit register to another 8 bit register
    fn Transfer(comptime dst: u3, comptime src: union(enum) { ir: u3, reg: State.Index }) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, _: *Pins) void {
                if (stage == 0) {
                    const from = switch (src) {
                        .ir => |idx| state.idx(r8(extract(ir, idx, 3))).*,
                        .reg => |reg| state.idx(reg).*,
                    };
                    state.idx(r8(extract(ir, dst, 3))).* = from;
                }
            }
        };
    }

    /// Do multiple operations in one machine cycle
    fn Join(comptime ops: anytype) type {
        return struct {
            pub fn op(comptime ir: u8, comptime stage: u2, state: *State, pins: *Pins) void {
                inline for (ops) |code| {
                    code.op(ir, stage, state, pins);
                }
            }
        };
    }

    /// These are the actual encodings and microcode for each instruction
    /// This basically encodes the instruction set architecture
    const decoder = Decoder.init
        .instr("00000000", .{Fetch(.ir)}) // nop
        .instr("01xxxxxx", .{Join(.{ Fetch(.ir), Transfer(3, .{ .ir = 0 }) })}) // ld r8, 'r8
        .instr("00xxx110", .{
            Fetch(.{ .z = .l }),
            Join(.{ Fetch(.ir), Transfer(3, .{ .reg = .{ .z = .l } }) }),
        }) // ld r8, n
        .Build();
};

/// Simulate 1 clock cycle and update state and pins
pub inline fn tick(state: *State, pins: *Pins) void {
    state.exec.stage(state, pins);
}

/// Used to test code
const Tester = struct {
    /// Cpu state
    state: State,

    /// Cpu pins
    pins: Pins,

    /// What is in the rom area
    rom: [0x8000]u8,

    /// What is in the ram area
    ram: [0x2000]u8,

    /// Creates initial tester state
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

    /// Run the simulation n+4 ticks (due to first opcode loaded in always being NOP)
    fn run(this: *@This(), ticks: usize) void {
        var n: usize = 0;
        while (n < ticks + 4) : (n += 1) {
            tick(&this.state, &this.pins);

            // Respond to pins so that we can test expected state of cpu
            if (this.pins.addr < 0x8000) {
                if (this.pins.rd) {
                    this.pins.data = this.rom[this.pins.addr];
                }
            } else if (this.pins.addr >= 0xC000 and this.pins.addr < 0xDFFF) {
                if (this.pins.rd) {
                    this.pins.data = this.ram[this.pins.addr - 0xC000];
                } else if (this.pins.wr) {
                    this.ram[this.pins.addr - 0xC000] = this.pins.data;
                }
            }
        }
    }
};

test "nop" {
    var tester = Tester.init(&.{ 0x00, 0x42 });
    tester.run(4);
    try std.testing.expectEqual(2, tester.state.pc.w);
    try std.testing.expectEqual(0x42, tester.state.ir.b);
}

test "ld r8, 'r8" {
    var tester = Tester.init(&.{0b01_000_001}); // ld b, c
    tester.state.bc.b.l = 0x42;
    tester.run(4);
    try std.testing.expectEqual(0x42, tester.state.bc.b.h);
}

test "ld r8, n" {
    var tester = Tester.init(&.{ 0b00_111_110, 0x42 }); // ld a, 0x42
    tester.run(8);
    try std.testing.expectEqual(0x42, tester.state.af.b.h);
}
