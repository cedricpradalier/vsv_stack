
#include "obstacle_mapper/LocalMap.h"

using namespace obstacle_mapper;

void LocalMap::shift(const tf::Transform & Delta) {
    size_t ncells = x_size * y_size;
    std::vector<Cell> new_cells(ncells,Cell(new ColumnCell(z_origin,vertical_resolution,z_size)));
    for (size_t j=0;j<y_size;j++) {
        size_t line = j*x_size;
        float y = y_origin + j * horizontal_resolution;
        for (size_t i=0;i<x_size;i++) {
            float x = x_origin + i * horizontal_resolution;
            tf::Vector3 v(x,y,0.0);
            v = Delta * v;
            Cell & c = cell(v.getX(),v.getY());
            if (c) {
                new_cells[line+i] = c;
            }
        }
    }
    cells = new_cells;
}

void LocalMap::update(const std::vector<float> & heights, const std::vector<LevelType> & occupancy) {
    size_t ncells = x_size * y_size;
    assert(heights.size() == ncells);
    assert(occupancy.size() == ncells);
    for (size_t i=0;i<ncells;i++) {
        if (occupancy[i] == Unknown) continue;
        if (!isnan(heights[i])) {
            if (!cells[i].unique()) {
                cells[i].reset(new ColumnCell(cells[i]));
            }
            // if (occupancy[i] == Occupied) {
            //     printf("Setting cell %d as occupied\n",(int)i);
            // }
            cells[i]->update(heights[i]-vertical_resolution/2,heights[i]+vertical_resolution/2,occupancy[i]);
        }
    }
}

void LocalMap::plot_by_type(LevelType type, const std::string & filename) const {
    FILE * fp = fopen(filename.c_str(),"w");
    assert(fp);
    for (size_t j=0;j<y_size;j++) {
        size_t line = j*x_size;
        float y = y_origin + j * horizontal_resolution;
        for (size_t i=0;i<x_size;i++) {
            float x = x_origin + i * horizontal_resolution;
            cells[line + i]->plot_by_type(x,y,type,fp);
        }
    }
    fclose(fp);
}



