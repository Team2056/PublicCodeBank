#include "Utils/OPRTree.h"

OPRTree::OPRTree() {
}

/**
 * @brief Add a value to the tree
 * 
 * @param values 
 */
void OPRTree::InsertValues(std::vector<std::vector<double>> values) {
    for (int i = 0; i < (int)values.size(); i++) {
        treeMap.insert(values[i][0], values[i][1]);
    }
}

/**
 * @brief Return the value at the given key (linearly interpolated)
 * 
 * @param key The value to get
 * @return double 
 */
double OPRTree::GetValue(double key) {
    return treeMap.operator[](key);
}
