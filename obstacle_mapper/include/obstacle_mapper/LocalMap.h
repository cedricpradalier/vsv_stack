#ifndef LOCAL_MAP_H
#define LOCAL_MAP_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <obstacle_mapper/ColumnCell.h>
#include <tf/tf.h>

namespace obstacle_mapper {

    class LocalMap {
        public:
            typedef boost::shared_ptr<ColumnCell> Cell;

        protected:
            size_t x_size, y_size, z_size;
            float x_origin, y_origin, z_origin;
            float horizontal_resolution, vertical_resolution;

            std::vector<Cell> cells;
            Cell empty;

        protected:
            inline Cell & cell(float x, float y) {
                int ix, iy, i ;
                ix = (int)round((x-x_origin)/horizontal_resolution);
                iy = (int)round((y-y_origin)/horizontal_resolution);
                if ((ix < 0) || (iy < 0)) return empty;
                if ((ix >= (int)x_size) || (iy >= (int)y_size)) return empty;
                i = iy * x_size + ix;
                return cells[i];
            }

            inline const Cell & cell(float x, float y) const {
                int ix, iy, i ;
                ix = (int)round((x-x_origin)/horizontal_resolution);
                iy = (int)round((y-y_origin)/horizontal_resolution);
                if ((ix < 0) || (iy < 0)) return empty;
                if ((ix >= (int)x_size) || (iy >= (int)y_size)) return empty;
                i = iy * x_size + ix;
                return cells[i];
            }


        public:

            LocalMap(float xorg, float yorg, float zorg, float h_resolution, float v_resolution, 
                    size_t xsize, size_t ysize, size_t zsize) : x_size(xsize), y_size(ysize), z_size(zsize),
            x_origin(xorg), y_origin(yorg), z_origin(zorg), horizontal_resolution(h_resolution), vertical_resolution(v_resolution) {
                size_t ncells = x_size * y_size;
                cells = std::vector<Cell>(ncells,Cell(new ColumnCell(z_origin,vertical_resolution,z_size)));
            }

            void update(float x, float y, float zstart, float zend, LevelType type) {
                Cell & c = cell(x,y);
                if (!c) return;
                if (!c.unique()) {
                    // Copy on write
                    c.reset(new ColumnCell(c));
                }
                c->update(zstart,zend,type);
            }

            void update(const std::vector<float> & heights, const std::vector<LevelType> & occupancy);

            inline LevelType operator()(float x, float y, float z) const {
                const Cell & c = cell(x,y);
                if (!c) return Unknown;
                return (*c)(z);
            }

            void shift(const tf::Transform & Delta);

            void plot_by_type(LevelType type, const std::string & filename) const;
    };
};




#endif // LOCAL_MAP_H
