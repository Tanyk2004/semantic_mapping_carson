// OcTreeKeyHash.h
#pragma once
#include <functional>
#include <octomap/OcTreeKey.h>

namespace std {

    // Specialize the std::hash template for octomap::point3d.
    template <>
    struct hash<octomap::point3d> {
        std::size_t operator()(const octomap::point3d& p) const {
            // Hash each component (x, y, z)
            std::size_t hash = 0;
            hash_combine(hash, std::hash<double>{}(p.x()));
            hash_combine(hash, std::hash<double>{}(p.y()));
            hash_combine(hash, std::hash<double>{}(p.z()));
            return hash;
        }

        // Helper function to combine hash values
        template <class T>
        void hash_combine(std::size_t& seed, const T& value) const {
            seed ^= std::hash<T>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
    };

};
