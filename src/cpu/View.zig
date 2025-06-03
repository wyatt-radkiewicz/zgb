const std = @import("std");

regs: Regs,
pins: Pins,

pub const reset = @This(){
    .pins = .reset,
    .regs = .reset,
};

pub const Regs = struct {
    bc: Reg = .zero,
    de: Reg = .zero,
    hl: Reg = .zero,
    af: Reg = .zero,
    pc: Reg = .zero,
    sp: Reg = .zero,
    ir: u8 = 0,
    ie: u8 = 0,

    pub const reset = @This(){};
};

pub const Reg = packed union {
    flag: packed struct {
        reserved: u4,
        carry: bool,
        bcd_half_carry: bool,
        bcd_subtract: bool,
        zero: bool,
    },
    byte: packed struct {
        l: u8,
        h: u8,
    },
    word: u16,

    pub const zero = @This(){
        .word = 0,
    };

    pub fn format(
        self: @This(),
        comptime _: []const u8,
        _: std.fmt.FormatOptions,
        writer: anytype,
    ) !void {
        try writer.print("0x{X:0>4}", .{self.word});
    }
};

pub const Pins = packed struct {
    addr: u16 = 0,
    data: u8 = 0,

    rd: bool = true,
    wr: bool = false,
    cs: bool = true,

    pub const reset = @This(){};
};
