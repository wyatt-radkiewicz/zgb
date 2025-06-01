regs: Regs,
pins: Pins,

pub const reset = @This(){
    .pins = .reset,
    .regs = .reset,
};

pub const Regs = struct {
    bc: Reg,
    de: Reg,
    hl: Reg,
    af: Reg,
    pc: Reg,
    sp: Reg,
    ir: u8,
    ie: u8,

    pub const reset = @This(){
        .bc = .zero,
        .de = .zero,
        .hl = .zero,
        .af = .zero,
        .pc = .zero,
        .sp = .zero,
        .ir = 0,
        .ie = 0,
    };
};

pub const Reg = packed union {
    byte: packed struct {
        l: u8,
        h: u8,
    },
    word: u16,

    pub const zero = @This(){
        .word = 0,
    };
};

pub const Pins = packed struct {
    addr: u16,
    data: u8,

    rd: bool,
    wr: bool,
    cs: bool,

    pub const reset = @This(){
        .addr = 0,
        .data = 0,
        .rd = true,
        .wr = true,
        .cs = true,
    };
};
