const std = @import("std");

const Isa = @import("Isa.zig");
pub const View = @import("View.zig");

view: View,
exec: Isa.Microcode.Ptr,

pub const reset = @This(){
    .view = .reset,
    .exec = decoder[0x00],
};

pub inline fn tick(this: *@This()) void {
    this.*.exec = Isa.Microcode.call(this.*.exec, &this.*.view, &decoder);
}

const decoder = Isa.decoder(&.{
    .instr("00000000", .{Fetch}),
    .instr("xxxxxxxx", .{Stub}),
});

const Stub = struct {
    pub inline fn op(comptime stage: u2, view: *View) void {
        _ = stage;
        _ = view;
    }
};

fn Read(comptime to: []const u8, assert_addr: fn (*View) u16) type {
    return struct {
        pub fn op(comptime stage: u2, view: *View) void {
            switch (stage) {
                0 => {
                    view.*.pins.addr = assert_addr(view);
                    view.*.pins.rd = true;
                    view.*.pins.wr = false;
                    view.*.pins.cs = true;
                },
                1 => {
                    view.*.pins.cs = switch (view.*.pins.addr) {
                        0xA000...0xFDFF => false,
                        else => true,
                    };
                },
                2 => {},
                3 => {
                    @field(view.*.regs, to) = view.*.pins.data;
                },
            }
        }
    };
}

const Fetch = struct {
    pub fn assertAddr(view: *View) u16 {
        view.*.regs.pc.word += 1;
        return view.*.regs.pc.word;
    }

    pub fn op(comptime stage: u2, view: *View) void {
        Read("ir", assertAddr).op(stage, view);
    }
};

fn tester(timeout: usize, code: []const u8, expect_views: anytype) !void {
    var cpu = reset;
    var timer: usize = 0;
    const ram = [1]u8{0} ** 0x8000;

    while (timer != timeout) : (timer += 1) {
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
        if (comptime std.mem.indexOfScalar(u8, name, '_')) |idx| {
            const reg_name = name[0..idx];
            if (!@hasField(View.Regs, reg_name)) {
                @compileError("No register found by the name of " ++ reg_name);
            } else if (name.len - idx <= 1) {
                @compileError("Register with '_' but no byte field access");
            }

            const reg = @field(cpu.view.regs, reg_name);
            try std.testing.expectEqual(@as(u8, expected), switch (name[idx + 1]) {
                'l' => reg.byte.l,
                'h' => reg.byte.h,
                else => @compileError("No register field found by the name " ++ name[idx + 1 ..]),
            });
        } else {
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
            } else {
                @compileError("No register or pin found by the name " ++ name);
            }
        }
    }
}

test "nop" {
    try tester(4, &.{ 0x00, 0x42 }, .{
        .addr = 1,
        .data = 0x42,
        .cs = true,
        .rd = true,
        .wr = false,

        .pc = 1,
        .ir = 0x42,
    });
}
