//
// Created by xlz on 17-11-4.
//
#include "map_offset.h"
#include <stdint.h>
#include <common/nature.h>

namespace TiEV{
    //dist + -> image move upward(laser coordinate to back-axis coordinate)
    //dist - -> image move dowmward(laser coordinate to forward-axis coordinate)
    void array_trans(uint8_t array_result[401][151], uint8_t cells[401][151],float dist){
        int offset = round(dist / TiEV::GRID_RESOLUTION) ;
        if(offset > 0)
        {
            for(int m=0 ; m<TiEV::GRID_ROW - offset;m++)
                for(int n=0;n<TiEV::GRID_COL;n++)
                {
                    array_result[m][n] = cells[ m + offset ][n];
                }
            for(int m=TiEV::GRID_ROW - offset ; m<TiEV::GRID_ROW;m++)
                for(int n=0;n<TiEV::GRID_COL;n++)
                {
                    array_result[m][n] = 0 ;
                }
        }
        else
        {
            offset = -offset;
            for(int m=0 ; m<offset;m++)
                for(int n=0;n<TiEV::GRID_COL;n++)
                {
                    array_result[m][n] = 0 ;
                }
            for(int m=offset ; m<TiEV::GRID_ROW;m++)
                for(int n=0;n<TiEV::GRID_COL;n++)
                {
                    array_result[m][n] = cells[ m - offset ][n]; ;
                }
        }

    }
}