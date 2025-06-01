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

pub const decoder = Isa.decoder(&.{
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

test "nop" {
    var cpu = reset;
    cpu.tick();
    try std.testing.expectEqual(View.Pins{
        .addr = 1,
        .data = 0,
        .cs = true,
        .rd = true,
        .wr = false,
    }, cpu.view.pins);
    cpu.tick();
    cpu.tick();
    cpu.tick();
}
