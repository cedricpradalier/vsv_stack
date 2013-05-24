#ifndef MULTI_LEVEL_CELL_H
#define MULTI_LEVEL_CELL_H

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <map>

namespace obstacle_mapper {
    
    // Not yet working. Too much tests to do it quickly and efficiently.

    class MultiLevelCell {
        public:
            typedef enum {Unknown, Free, Occupied} LevelType;
        protected:
            struct Level {
                float start;
                LevelType type;
                Level() : start(NAN), type(Unknown) {}
                Level(float s, LevelType t) : start(s), type(t) {}
            };
            typedef std::map< float,LevelType,std::less<float> > LevelMap;
            LevelMap levels;
    
        public:
            MultiLevelCell() {}

            inline void update(float start, float end, LevelType type) {
                assert(start < end);
                if (levels.empty()) {
                    levels[start] = type;
                    levels[end] = Unknown;
                    return;
                }
                LevelMap::iterator it_start = levels.upper_bound(start);
                LevelMap::iterator it_end = levels.upper_bound(end);
                if ((it_end == levels.begin()) || (it_start == levels.end())) {
                    // interval is before or after the map
                    levels[start] = type;
                    levels[end] = Unknown;
                    return;
                }
                if (it_start == it_end) {
                    // interval is within an existing segment
                    it_start --;
                    levels[start] = type;
                    levels[end] = it_start->second;
                    return;
                }
                // interval is overlapping existing segments
                if (it_start != levels.begin()) {
                    LevelMap::iterator it_begin = it_start; 
                    it_begin --;
                    if (it_begin->second != type) {
                        levels[start] = type;
                    }
                    LevelType type_end = it_end->second;
                    levels.erase(it_start,it_end);
                    levels[start] = type;
                    it_end --;
                    levels[end] = type_end;
                } else {
                    LevelType type_end = it_end->second;
                    levels.erase(it_start,it_end);
                    levels[start] = type;
                    it_end --;
                    levels[end] = type_end;
                }
            }

            void print() const {
                printf("U ");
                for (LevelMap::const_iterator it = levels.begin();
                        it != levels.end() ; it++) {
                    printf("%.3f ",it->first);
                    switch (it->second) {
                        case Free: 
                            printf("F "); 
                            break;
                        case Occupied: 
                            printf("O "); 
                            break;
                        case Unknown: 
                        default: 
                            printf("U "); 
                            break;
                    }
                }
            }

            bool validate() const {
                LevelMap::const_iterator it, itp;
                it = levels.begin();
                itp = it; it++;
                while (it != levels.end()) {
                   if (it->second == itp->second) {
                       return false;
                   }
                   itp = it;
                   it++; 
                }
                return true;
            }
    };

};


#endif // MULTI_LEVEL_CELL_H
