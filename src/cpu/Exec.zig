const std = @import("std");

const View = @import("View.zig");

const Exec = @This();

pfn: Microcode,
lut: *const [256]Microcode,
tmp: Buffer,

pub fn init(lut: *const [256]Microcode) Exec {
    return .{
        .pfn = lut[0x00],
        .lut = lut,
        .tmp = .zero,
    };
}

pub fn decoder(comptime decoders: []const Decoder) [256]Microcode {
    // Order the decoders to add the most specific ones last
    var priority: [decoders.len]Decoder = undefined;
    @memcpy(&priority, decoders);
    std.sort.insertion(Decoder, &priority, {}, struct {
        fn lessThan(_: void, lhs: Decoder, rhs: Decoder) bool {
            return lhs.opcodes.count() < rhs.opcodes.count();
        }
    }.lessThan);

    // Now apply the decoders to a opcode-decoder, and make sure no matches leak
    var lut: [256]Microcode = undefined;
    inline for (&lut, 0..) |*pfn, byte| {
        inline for (priority) |decode| {
            if (decode.opcodes.isSet(byte)) {
                pfn.* = decode.microcode;
                break;
            }
        } else {
            @compileError(std.fmt.comptimePrint("Decode LUT leaked opcode 0x{X:0>2}", .{byte}));
        }
    }
    return lut;
}

pub const Buffer = struct {
    z: u8,

    pub const zero = @This(){
        .z = 0,
    };
};

pub const Microcode = *const fn (*Exec, *View) void;

pub const Decoder = struct {
    opcodes: std.StaticBitSet(256),
    microcode: Microcode,

    pub fn init(comptime enc: *const [8]u8, code: Microcode) @This() {
        @setEvalBranchQuota(1000000);

        comptime var set: u8, var any: u8 = .{ 0, 0 };
        inline for (enc, 0..) |char, idx| {
            set |= switch (char) {
                '1' => 0x80 >> idx,
                else => 0,
            };
            any |= switch (char) {
                '0', '1' => 0x80 >> idx,
                else => 0,
            };
        }

        comptime var mask = std.StaticBitSet(256).initEmpty();
        inline for (0..256) |byte| {
            mask.setValue(byte, byte ^ set & any == 0);
        }

        return .{
            .opcodes = mask,
            .microcode = code,
        };
    }

    // Link a list of m-states together into 1 instruction, and add a decode/jump at the end
    pub fn instr(comptime enc: *const [8]u8, comptime ops: anytype) @This() {
        // Link them all together (starting at the end)
        comptime var op = ops.len - 1;
        comptime var last: ?Microcode = null;
        const code = while (true) : (op -= 1) {
            // Link the stages together
            comptime var stage = 3;
            inline while (true) : (stage -= 1) {
                last = @ptrCast(&(struct {
                    fn inner(exec: *Exec, view: *View) void {
                        ops[op].op(stage, exec, view);
                        exec.*.pfn = last orelse exec.*.lut[view.regs.ir];
                    }
                }.inner));

                if (stage == 0) {
                    break;
                }
            }

            if (op == 0) {
                break last orelse unreachable;
            }
        };

        return .init(enc, code);
    }
};
