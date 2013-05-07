#ifndef SPLINE_REGISTER_H
#define SPLINE_REGISTER_H

#include <string>
#include <vector>
#include <stdio.h>
#include "SplinePath.h"

namespace splines {
	class SplineRegister
	{
		public:
			SplineRegister();
			~SplineRegister();

			bool load(const std::string & filename);
			bool save(const std::string & filename, bool append=false) const;
			void clear();

			void printList(FILE *out, bool all=false) const;

			const SplinePath * getPathByName(const std::string & s) const;
			const SplinePath * getPathById(unsigned int id) const;
			std::string getPathName(unsigned int id) const {
				if (id > names.size()) {
					return std::string();
				}
				return names[id];
			}

			unsigned int size() const {
				return paths.size();
			}
			const std::vector<std::string> & getPathNames() const {
				return names;
			}
			const std::vector<SplinePath*> & getPaths() const {
				return paths;
			}

			bool addPath(const std::string & name, const std::vector<Point> & points, double velocity);
			bool addPath(const std::string & name, const std::vector<Point> & points, const std::vector<double> & velocities);

		protected:
			double max_inter_points, min_inter_points;
		public:
			void setDecimationParams(double min_inter, double max_inter) {
				max_inter_points = max_inter;
				min_inter_points = min_inter;
			}

		protected:
			typedef std::vector<std::string> NameMap;
			typedef std::vector<SplinePath*> PathMap;
			PathMap paths;
			NameMap names;
	};

}
#endif // SPLINE_REGISTER_H
