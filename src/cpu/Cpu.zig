const std = @import("std");

const Exec = @import("Exec.zig");
pub const View = @import("View.zig");

view: View,
exec: Exec,

pub const reset = @This(){
    .view = .reset,
    .exec = .init(&Exec.decoder(&.{
        .instr("00000000", .{Fetch(toir)}), // nop
        .instr("01xxxxx0", .{struct {
            pub fn op(comptime stage: u2, exec: *Exec, view: *View) void {
                Fetch(toir).op(stage, exec, view);
                if (stage != 0) return;
                r8(view, 3).* = r8(view, 0).*;
            }
        }}), // ld r8, r8
        .instr("00xxx110", .{ Fetch(toz), struct {
            pub fn op(comptime stage: u2, exec: *Exec, view: *View) void {
                Fetch(toir).op(stage, exec, view);
                if (stage != 0) return;
                r8(view, 3).* = exec.tmp.z;
            }
        } }), // ld r8, imm8
        .instr("00110110", .{Fetch(toz), struct {
            pub fn op(comptime stage: u2, exec: *Exec, view: *View) void {
                switch (stage) {
                    0 => wrbus(stage, view, view.*.regs.hl.word),
                    1, 3 => wrbus(stage, view, {}),
                    2 => wrbus(stage, view, exec.*.tmp.z),
                }
            }
        }}), // ld (HL), imm8
        .instr("01110xxx", .{struct {
            pub fn op(comptime stage: u2, _: *Exec, view: *View) void {
                switch (stage) {
                    0 => wrbus(stage, view, view.*.regs.hl.word),
                    1, 3 => wrbus(stage, view, {}),
                    2 => wrbus(stage, view, r8(view, 0).*),
                }
            }
        }}), // ld (HL), r8
        .instr("01xxx110", .{struct {
            pub fn op(comptime stage: u2, _: *Exec, view: *View) void {
                switch (stage) {
                    0 => rdbus(stage, view, view.*.regs.hl.word),
                    1, 2 => rdbus(stage, view, {}),
                    3 => r8(view, 3).* = rdbus(stage, view, {}),
                }
            }
        }}), // ld r8, (HL)
        .instr("xxxxxxxx", .{Idle}), // stub
    })),
};

pub inline fn tick(this: *@This()) void {
    this.*.exec.pfn(&this.exec, &this.*.view);
}

fn cspin(addr: u16) bool {
    return switch (addr) {
        0xA000...0xFDFF => false,
        else => true,
    };
}

fn rdbus(comptime stage: u2, view: *View, options: if (stage == 0) u16 else void) switch (stage) {
    3 => u8,
    else => void,
} {
    switch (stage) {
        0 => {
            view.*.pins.addr = options;
            view.*.pins.rd = true;
            view.*.pins.wr = false;
            view.*.pins.cs = true;
        },
        1 => view.*.pins.cs = cspin(view.*.pins.addr),
        2 => {},
        3 => return view.*.pins.data,
    }
}

fn wrbus(comptime stage: u2, view: *View, options: switch (stage) {
    0 => u16,
    1, 3 => void,
    2 => u8,
}) void {
    switch (stage) {
        0 => {
            view.*.pins.addr = options;
            view.*.pins.rd = false;
            view.*.pins.wr = false;
            view.*.pins.cs = true;
        },
        1 => view.*.pins.cs = cspin(view.*.pins.addr),
        2 => {
            view.*.pins.wr = true;
            view.*.pins.data = options;
        },
        3 => {},
    }
}

fn r8(view: *View, comptime ir_idx: u3) *u8 {
    return switch (@as(u3, @truncate(view.*.regs.ir >> ir_idx))) {
        0 => &view.*.regs.bc.byte.h,
        1 => &view.*.regs.bc.byte.l,
        2 => &view.*.regs.de.byte.h,
        3 => &view.*.regs.de.byte.l,
        4 => &view.*.regs.hl.byte.h,
        5 => &view.*.regs.hl.byte.l,
        6 => unreachable,
        7 => &view.*.regs.af.byte.h,
    };
}

const Idle = struct {
    pub inline fn op(comptime stage: u2, _: *Exec, view: *View) void {
        if (stage == 0) {
            view.*.pins.rd = true;
            view.*.pins.wr = false;
            view.*.pins.cs = true;
        }
    }
};

fn toir(_: *Exec, view: *View, byte: u8) void {
    view.*.regs.ir = byte;
}

fn toz(exec: *Exec, _: *View, byte: u8) void {
    exec.*.tmp.z = byte;
}

fn Fetch(comptime to: fn (exec: *Exec, view: *View, byte: u8) void) type {
    return struct {
        pub fn op(comptime stage: u2, exec: *Exec, view: *View) void {
            switch (stage) {
                0 => {
                    rdbus(stage, view, view.*.regs.pc.word);
                    view.*.regs.pc.word += 1;
                },
                1, 2 => rdbus(stage, view, {}),
                3 => to(exec, view, rdbus(stage, view, {})),
            }
        }
    };
}

fn tester(timeout: usize, code: []const u8, initial_view: ?View, expect_views: anytype) !void {
    var cpu = reset;
    var timer: usize = 0;
    const ram = [1]u8{0} ** 0x8000;

    if (initial_view) |view| {
        cpu.view = view;
    }
    while (timer < timeout + 4) : (timer += 1) {
        // Tick the cpu
        cpu.tick();

        // Observe the pins and respond
        switch (cpu.view.pins.addr) {
            0x0000...0x7FFF => if (cpu.view.pins.rd) {
                cpu.view.pins.data = if (cpu.view.pins.addr < code.len) code[cpu.view.pins.addr] else 0;
            },
            0x8000...0xFFFF => if (cpu.view.pins.rd) {
                cpu.view.pins.data = ram[cpu.view.pins.addr - 0x8000];
            },
        }
    }

    const ExpectType = @TypeOf(expect_views);
    inline for (std.meta.fields(ExpectType)) |field| {
        const name = field.name;
        const expected = @field(expect_views, name);
        if (@hasField(View.Regs, name)) {
            const reg = @field(cpu.view.regs, name);
            if (@TypeOf(reg) == View.Reg) {
                try std.testing.expectEqual(expected, reg.word);
            } else {
                try std.testing.expectEqual(@as(@TypeOf(reg), expected), reg);
            }
        } else if (@hasField(View.Pins, name)) {
            const pins = @field(cpu.view.pins, name);
            try std.testing.expectEqual(@as(@TypeOf(pins), expected), pins);
        } else if (comptime std.meta.stringToEnum(enum { a, b, c, d, e, f, h, l }, name)) |reg| {
            switch (reg) {
                .b => try std.testing.expectEqual(expected, cpu.view.regs.bc.byte.h),
                .c => try std.testing.expectEqual(expected, cpu.view.regs.bc.byte.l),
                .d => try std.testing.expectEqual(expected, cpu.view.regs.de.byte.h),
                .e => try std.testing.expectEqual(expected, cpu.view.regs.de.byte.l),
                .h => try std.testing.expectEqual(expected, cpu.view.regs.hl.byte.h),
                .l => try std.testing.expectEqual(expected, cpu.view.regs.hl.byte.l),
                .a => try std.testing.expectEqual(expected, cpu.view.regs.af.byte.h),
                .f => try std.testing.expectEqual(expected, cpu.view.regs.af.byte.l),
            }
        } else {
            @compileError("No register or pin found by the name " ++ name);
        }
    }
}

test "nop" {
    try tester(4, &.{ 0x00, 0x42 }, null, .{
        .addr = 1,
        .data = 0x42,
        .cs = true,
        .rd = true,
        .wr = false,

        .pc = 2,
        .ir = 0x42,
    });
}

test "ld r8, r8" {
    try tester(4, &.{0b0111_1000}, View{
        .pins = .reset,
        .regs = .{
            .bc = .{ .word = 0x4200 },
        },
    }, .{
        .a = 0x42,
        .b = 0x42,
    });
}

test "ld r8, imm8" {
    try tester(8, &.{ 0b0011_1110, 0x13 }, null, .{ .a = 0x13 });
}

test "ld r8, (HL)" {
    try tester(4, &.{ 0b0111_1110 }, View{
        .pins = .reset,
        .regs = .{
            .hl = .{ .word = 0x0001 },
        },
    }, .{
        .a = 0x00, // Should load value at memory location 0x0001, which is 0x00
    });
}

test "ld (HL), r8" {
    try tester(4, &.{ 0b0111_0000 }, View{
        .pins = .reset,
        .regs = .{
            .hl = .{ .word = 0xC000 },
            .bc = .{ .byte = .{ .h = 0x42, .l = 0x00 } },
        },
    }, .{
        .addr = 0xC000, // Should write to memory location 0xC000
        .data = 0x42,   // Should write value 0x42 (from B register)
        .wr = true,     // Should be a write operation
    });
}

test "ld (HL), n" {
    try tester(8, &.{ 0x36, 0x42 }, View{
        .pins = .reset,
        .regs = .{
            .hl = .{ .word = 0xC000 },
        },
    }, .{
        .addr = 0xC000, // Should write to memory location 0xC000
        .data = 0x42,   // Should write immediate value 0x42
        .wr = true,     // Should be a write operation
    });
}
