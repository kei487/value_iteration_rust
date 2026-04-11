#include "load_tiles.h"

void load_transitions(
    const ap_uint<32> *trans_table,
    offset_t delta_table[N_ACTIONS][N_THETA][3])
{
    LOAD_TRANS: for (int i = 0; i < TRANS_TABLE_SIZE; i++) {
        #pragma HLS PIPELINE II=1
        ap_uint<32> w = trans_table[i];
        int a = i / N_THETA;
        int t = i % N_THETA;
        delta_table[a][t][0] = (offset_t)(w(7, 0));    // dix
        delta_table[a][t][1] = (offset_t)(w(15, 8));   // diy
        delta_table[a][t][2] = (offset_t)(w(23, 16));  // dit
    }
}

void load_tile(
    const value_t *value_table,
    const penalty_t *penalty_table,
    value_t val_buf[TILE_H_H][TILE_W_H][N_THETA],
    penalty_t pen_buf_0[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_1[TILE_H_H][TILE_W_H],
    penalty_t pen_buf_2[TILE_H_H][TILE_W_H],
    int tile_ox, int tile_oy,
    int map_x, int map_y)
{
    // Load value table tile + halo
    LOAD_V_Y: for (int ly = 0; ly < TILE_H_H; ly++) {
        int gy = tile_oy - HALO + ly;  // global y coordinate

        LOAD_V_X: for (int lx = 0; lx < TILE_W_H; lx++) {
            int gx = tile_ox - HALO + lx;  // global x coordinate

            bool out_of_bounds = (gx < 0 || gx >= map_x || gy < 0 || gy >= map_y);

            // Load penalty (same for all 3 copies)
            penalty_t pen;
            if (out_of_bounds) {
                pen = PENALTY_OBSTACLE;
            } else {
                pen = penalty_table[gy * map_x + gx];
            }
            pen_buf_0[ly][lx] = pen;
            pen_buf_1[ly][lx] = pen;
            pen_buf_2[ly][lx] = pen;

            // Load value for all theta
            LOAD_V_T: for (int it = 0; it < N_THETA; it++) {
                #pragma HLS PIPELINE II=1
                if (out_of_bounds) {
                    val_buf[ly][lx][it] = MAX_VALUE;
                } else {
                    int addr = (gy * map_x + gx) * N_THETA + it;
                    val_buf[ly][lx][it] = value_table[addr];
                }
            }
        }
    }
}
