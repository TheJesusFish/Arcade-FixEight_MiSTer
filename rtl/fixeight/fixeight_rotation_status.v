// SPDX-License-Identifier: BSD-3-Clause

module fixeight_rotation_status (
    input      [63:0] status,
    output     [1:0] menu,
    output           framebuf_flip
);

// Lower-case o78 in CONF_STR addresses the extended status field [40:39].
// 00 rotates, 01 preserves the native raster, and 10 uses the framebuffer
// only for the alternate unrotated orientation.
assign menu = status[40:39];
assign framebuf_flip = menu == 2'b10;

endmodule
