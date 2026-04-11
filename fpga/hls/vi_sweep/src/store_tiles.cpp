#include "store_tiles.h"

void store_tile(
    value_t *value_table,
    const value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    int tile_ox, int tile_oy,
    int tile_w, int tile_h,
    int map_x)
{
    STORE_Y: for (int iy = 0; iy < tile_h; iy++) {
        int gy = tile_oy + iy;

        STORE_X: for (int ix = 0; ix < tile_w; ix++) {
            int gx = tile_ox + ix;
            int by = iy + HALO;
            int bx = ix + HALO;

            STORE_T: for (int it = 0; it < N_THETA; it++) {
                #pragma HLS PIPELINE II=1
                int addr = (gy * map_x + gx) * N_THETA + it;
                value_table[addr] = val_buf[by][bx][it];
            }
        }
    }
}
