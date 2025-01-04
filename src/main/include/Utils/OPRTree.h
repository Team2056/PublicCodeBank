#pragma once

#include <wpi/interpolating_map.h>

#include <vector>

class OPRTree {
   public:
    OPRTree();

    /**
     * @brief Takes 2D vector of doubles key, then value to put into the map
     * @param values
     */
    void InsertValues(std::vector<std::vector<double>> values);

    /**
     * @brief Returns the value associated with a given key. If there's no matching key, the value returned will be a linear interpolation between the keys before and after the provided one.
     * @param key - the key
     * @return double, the value
     */
    double GetValue(double key);

   private:
    wpi::interpolating_map<double, double> treeMap;
};