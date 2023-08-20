#pragma once
#include <glm/vec3.hpp>
#include <vector>
#include <array>

namespace MathUtils {
    glm::vec3 calculateTriangleNormal(const glm::vec3 triangleVertices[3]);
    std::vector<std::array<uint64_t, 3>> triangulatePolygon(const std::vector<uint32_t> &polygonVertices, const std::vector<glm::vec3>& positions);
    bool isPointInsideTriangle(const glm::vec3& projectedPoint, const glm::vec3 trianglePoints[3]);
}