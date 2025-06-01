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

const Fetch = struct {
    pub inline fn op(comptime stage: u2, view: *View) void {
        _ = view;
        switch (stage) {
            0 => {},
            1, 2 => {},
            3 => {},
        }
    }
};

test "nop" {
    var cpu = reset;
    cpu.tick();
    cpu.tick();
    cpu.tick();
    cpu.tick();
}
