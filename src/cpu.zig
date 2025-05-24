//! The cpu implementation for the DMG
const std = @import("std");

// Each instruction shows how to:
// - serialize/deserialize it
// - encode/decode it
// - how may cpu states it takes
const instructions = [_]struct { []const u8, []const u8, usize }{
    // Block 0
    .{ "nop", "00000000", 4 },
    .{ "ld r16, imm16", "00dd0001", 4 },
    .{ "ld [r16mem], a", "00dd0010", 4 },
    .{ "ld a, [r16mem]", "00ss1010", 4 },
    .{ "ld [imm16], sp", "00001000", 4 },
    .{ "inc r16", "00oo0011", 4 },
    .{ "dec r16", "00oo1011", 4 },
    .{ "add hl, r16", "00oo1001", 4 },
    .{ "inc r8", "00ooo100", 4 },
    .{ "dec r8", "00ooo101", 4 },
    .{ "ld r8, imm8", "00ooo110", 4 },
    .{ "rlca", "00000111", 4 },
    .{ "rrca", "00001111", 4 },
    .{ "rla", "00010111", 4 },
    .{ "rra", "00011111", 4 },
    .{ "daa", "00100111", 4 },
    .{ "cpl", "00101111", 4 },
    .{ "scf", "00110111", 4 },
    .{ "ccf", "00111111", 4 },
    .{ "jr imm8", "00011000", 4 },
    .{ "jr cond, imm8", "001cc000", 4 },
    .{ "stop", "00010000", 4 },

    // Block 1: 8-bit register-to-register loads
    .{ "ld r8, r8", "01dddsss", 4 },
    .{ "halt", "01110110", 4 },

    // Block 2: 8-bit arithmetic
    .{ "add a, r8", "10000ooo", 4 },
    .{ "adc a, r8", "10001ooo", 4 },
    .{ "sub a, r8", "10010ooo", 4 },
    .{ "sbc a, r8", "10011ooo", 4 },
    .{ "and a, r8", "10100ooo", 4 },
    .{ "xor a, r8", "10101ooo", 4 },
    .{ "or a, r8", "10110ooo", 4 },
    .{ "cp a, r8", "10111ooo", 4 },

    // Block 3
    .{ "add a, imm8", "11000110", 4 },
    .{ "adc a, imm8", "11001110", 4 },
    .{ "sub a, imm8", "11010110", 4 },
    .{ "sbc a, imm8", "11011110", 4 },
    .{ "and a, imm8", "11100110", 4 },
    .{ "xor a, imm8", "11101110", 4 },
    .{ "or a, imm8", "11110110", 4 },
    .{ "cp a, imm8", "11111110", 4 },
    .{ "ret cond", "110cc000", 4 },
    .{ "ret", "11001001", 4 },
    .{ "reti", "11011001", 4 },
    .{ "jp cond, imm16", "110cc010", 4 },
    .{ "jp imm16", "11000011", 4 },
    .{ "jp hl", "11101001", 4 },
    .{ "call cond, imm16", "110cc100", 4 },
    .{ "call imm16", "11001101", 4 },
    .{ "rst tgt3", "11ttt111", 4 },
    .{ "pop r16stk", "11rr0001", 4 },
    .{ "push r16stk", "11rr0101", 4 },
    .{ "prefix", "11001011", 4 },
    .{ "ldh [c], a", "11100010", 4 },
    .{ "ldh [imm8], a", "11100000", 4 },
    .{ "ld [imm16], a", "11101010", 4 },
    .{ "ldh a, [c]", "11110010", 4 },
    .{ "ldh a, [imm8]", "11110000", 4 },
    .{ "ld a, [imm16]", "11111010", 4 },
    .{ "add sp, imm8", "11101000", 4 },
    .{ "ld hl, sp + imm8", "11111000", 4 },
    .{ "ld sp, hl", "11111001", 4 },
    .{ "di", "11110011", 4 },
    .{ "ei", "11111011", 4 },

    // $CB prefix instructions
    .{ "rlc r8", "00000ooo", 4 }, // Note: These would be prefixed with CB
    .{ "rrc r8", "00001ooo", 4 },
    .{ "rl r8", "00010ooo", 4 },
    .{ "rr r8", "00011ooo", 4 },
    .{ "sla r8", "00100ooo", 4 },
    .{ "sra r8", "00101ooo", 4 },
    .{ "swap r8", "00110ooo", 4 },
    .{ "srl r8", "00111ooo", 4 },
    .{ "bit b3, r8", "01bbbooo", 4 },
    .{ "res b3, r8", "10bbbooo", 4 },
    .{ "set b3, r8", "11bbbooo", 4 },
};

// Pins you give to the CPU when ticking it and what you get back
// These don't represent voltage signals but rather the logic signals (pay no attention to active lo for example)
pub const Pins = struct {
    addr: u16,
    data: u8,

    rd: bool,
    wr: bool,
    cs: bool,

    pub const zero = @This(){
        .addr = 0x0000,
        .data = 0x00,
        .rd = false,
        .wr = false,
        .cs = false,
    };
};

pub fn Reg(comptime width: u16) type {
    return switch (width) {
        8 => enum {
            c,
            b,
            e,
            d,
            l,
            h,
            f,
            a,
            sp_l,
            sp_h,
            pc_l,
            pc_h,
            ir,
            ie,

            pub const int = u8;
        },
        16 => enum {
            bc,
            de,
            hl,
            af,
            sp,
            pc,

            pub const int = u16;
        },
        else => @compileError("Expected register width of 8 or 16"),
    };
}

// Helper utility to extract bits from an integer (inclusive)
inline fn extract(int: anytype, comptime from: std.math.Log2Int(@TypeOf(int)), comptime to: std.math.Log2Int(@TypeOf(int))) std.meta.Int(.unsigned, to - from + 1) {
    return @truncate(int >> from & (1 << to) - 1);
}

// The actual chip, so internal state and external pins
pub const Cpu = struct {
    proc: Proc,
    regs: [std.meta.tags(Reg(8)).*.len]u8 align(@alignOf(u16)),
    pins: Pins,

    // Cpu state initially
    pub const init = Cpu{
        .proc = Proc.decode(0),
        .regs = [_]u8{0} ** @typeInfo(@FieldType(@This(), "regs")).array.len,
        .pins = .zero,
    };

    // Tick the cpu state
    pub fn tick(this: *@This()) void {
        _ = switch (this.*.proc) {
            // nop
            .nop0 => this.fetch(0).to(.nop1),
            .nop1 => this.fetch(1).to(.nop2),
            .nop2 => this.fetch(2).to(.nop3),
            .nop3 => this.fetch(3),

            // ldr8r8
            .ldr8r80 => this.wrreg(8, this.r8(3), this.rdreg(8, this.r8(0))).fetch(0).to(.ldr8r81),
            .ldr8r81 => this.fetch(1).to(.ldr8r81),
            .ldr8r82 => this.fetch(2).to(.ldr8r82),
            .ldr8r83 => this.fetch(3),
        };
    }

    // This is a fetch cycle op
    inline fn fetch(this: *@This(), comptime cycle: u2) *@This() {
        return switch (cycle) {
            0 => this.rdbus(0, this.rdreg(16, .pc)).addimm(16, .pc, 1),
            1 => this.rdbus(1, {}),
            2 => this.rdbus(2, {}),
            3 => this.rdbus(3, .ir).to(Proc.decode(this.rdreg(8, .ir))),
        };
    }

    // This is read cycle
    inline fn rdbus(this: *@This(), comptime cycle: u2, options: switch (cycle) {
        0 => u16,
        1, 2 => void,
        3 => Reg(8),
    }) *@This() {
        return switch (cycle) {
            0 => blk: {
                this.*.pins.addr = options;
                this.*.pins.rd = true;
                this.*.pins.wr = false;
                this.*.pins.cs = true;
                break :blk this;
            },
            1 => blk: {
                this.*.pins.cs = switch (this.*.pins.addr) {
                    0xA000...0xFDFF => true,
                    else => false,
                };
                break :blk this;
            },
            2 => this,
            3 => this.wrreg(8, options, this.*.pins.data),
        };
    }

    // Goes to the specified procedure
    inline fn to(this: *@This(), proc: Proc) *@This() {
        this.*.proc = proc;
        return this;
    }

    // Performs an add operation on a register with an immediate
    inline fn addimm(this: *@This(), comptime width: u16, reg: Reg(width), imm: Reg(width).int) *@This() {
        return this.wrreg(width, reg, this.rdreg(width, reg) +% imm);
    }

    // Read register
    inline fn rdreg(this: *const @This(), comptime width: u16, reg: Reg(width)) Reg(width).int {
        return switch (width) {
            8 => this.*.regs[@intFromEnum(reg)],
            16 => std.mem.readInt(u16, this.*.regs[@intFromEnum(reg)..][0..2], .little),
            else => unreachable,
        };
    }

    // Write register
    inline fn wrreg(this: *@This(), comptime width: u16, reg: Reg(width), val: Reg(width).int) *@This() {
        switch (width) {
            8 => this.*.regs[@intFromEnum(reg)] = val,
            16 => std.mem.writeInt(u16, this.*.regs[@intFromEnum(reg)..][0..2], val, .little),
            else => unreachable,
        }
        return this;
    }

    // Decode r8 register from instruction register (f is decoded from [hl])
    inline fn r8(this: *const @This(), comptime idx: std.math.Log2Int(u8)) Reg(8) {
        return @enumFromInt(extract(this.rdreg(8, .ir), idx, idx + 2));
    }
};

// Different code run each tick
const Proc = enum {
    nop0,
    nop1,
    nop2,
    nop3,

    ldr8r80,
    ldr8r81,
    ldr8r82,
    ldr8r83,

    // Decode a procedure index from a byte
    fn decode(byte: u8) @This() {
        return (comptime blk: {
            // Define every procedure as a pattern.
            // Patterns defined later override ones defined earlier.
            const patterns = [_]struct { []const u8, @This() }{
                .{ "xxxxxxxx", .nop0 },
                .{ "01xxxxxx", .ldr8r80 },
                .{ "01110110", .nop0 }, // TODO: halt
            };

            // Look up table used to decode opcodes
            var lut = [_]@This(){.nop0} ** (std.math.maxInt(u8) + 1);
            for (patterns) |pattern| {
                // Generate needs and don't care mask
                var need: u8 = 0;
                var mask: u8 = 0;
                for (pattern[0], 0..) |c, i| {
                    need |= if (c == '1') 1 << i else 0;
                    mask |= if (c != 'x') 1 << i else 0;
                }

                // Apply pattern to every byte
                for (&lut, 0..) |*proc, encoding| {
                    if (encoding & mask ^ need == 0) {
                        proc.* = pattern[1];
                    }
                }
            }
            break :blk lut;
        })[byte];
    }
};

// Test run instructions
// Code starts at 0x0000
// RAM starts at 0x8000
fn testCode(code: []const u8, timeout: usize, regs: anytype) Cpu {
    var cpu = Cpu.init;
    var tick: usize = 0;
    const ram: [1024 * 8]u8 = undefined;

    // Apply registers
    inline for (std.meta.fieldNames(@TypeOf(regs))) |field| {
        if (std.meta.stringToEnum(Reg(16), field)) |r16| {
            // Check if its a 16bit register
            cpu.wrreg(16, r16, @field(regs, field));
        } else if (std.meta.stringToEnum(Reg(8), field)) |r8| {
            // Check if its a 8bit register
            cpu.wrreg(8, r8, @field(regs, field));
        } else {
            @compileError("Please specifiy a correct register name");
        }
    }

    // Run the CPU
    while (tick != timeout) {
        cpu.tick();
        tick += 1;

        // Put the write data on the wire
        if (cpu.pins.addr <= 0x7FFF and !cpu.pins.cs) {
            // Read from ROM
            if (cpu.pins.rd) {
                if (cpu.pins.addr < code.len) {
                    cpu.pins.data = code[cpu.pins.addr];
                }
            }
        } else if (cpu.pins.addr >= 0x8000 and cpu.pins.addr <= 0xBFFF and !cpu.pins.cs) {
            // Read from RAM
            if (cpu.pins.rd) {
                cpu.pins.data = ram[cpu.pins.addr];
            }
        }
    }
    return cpu;
}

test "nop" {
    const cpu = testCode(&.{0x42}, 4, .{});
    try std.testing.expectEqual(0x01, cpu.rdreg(16, .pc));
    try std.testing.expectEqual(0x42, cpu.rdreg(8, .ir));
}
