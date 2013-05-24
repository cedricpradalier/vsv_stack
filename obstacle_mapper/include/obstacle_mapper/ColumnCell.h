#ifndef COLUMN_CELL_H
#define COLUMN_CELL_H

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <map>
#include <boost/shared_ptr.hpp>

namespace obstacle_mapper {
    
    // Not yet working. Too much tests to do it quickly and efficiently.
    typedef enum {Unknown, Free, Occupied} LevelType;

    class ColumnCell {
        protected:
            float z0, resolution;
            std::vector<LevelType> column;
    
        public:
            ColumnCell(float zstart, float res, size_t num_cells) : z0(zstart), resolution(res), column(num_cells,Unknown) {}
            ColumnCell(const ColumnCell & c) : z0(c.z0), resolution(c.resolution), column(c.column) {}
            ColumnCell(const boost::shared_ptr<ColumnCell> & c) : z0(c->z0), resolution(c->resolution), column(c->column) {}

            inline void update(float start, float end, LevelType type) {
                int istart,iend,i;
                istart = std::max((int)round((start-z0)/resolution),0);
                iend = std::min((int)round((end-z0)/resolution),(int)column.size());
                // if (type == Occupied) {
                //      printf("Updating from %d to %d with type %d\n",istart,iend,type);
                // }
                if (istart==iend) {
                    column[istart] = type;
                } else {
                    for (i=istart;i<iend;i++) {
                        column[i] = type;
                    }
                }
            }

            inline LevelType operator()(float z) {
                int i = (int)round((z-z0)/resolution);
                if (i < 0) return Unknown;
                if (i >= (signed)column.size()) return Unknown;
                return column[i];
            }

            void plot_by_type(float x, float y, LevelType type, FILE * fp) const {
                for (size_t i=0;i<column.size();i++) {
                    if (column[i] == type) {
                        fprintf(fp,"%e %e %e\n",x,y,z0+i*resolution);
                    }
                }
            }
    };

};


#endif // COLUMN_CELL_H
