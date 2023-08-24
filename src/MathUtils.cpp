#include "MathUtils.h"
#include <glm/geometric.hpp>
#include <CGAL/Advancing_front_surface_reconstruction.h>
#include "CGAL/Simple_cartesian.h"

glm::vec3 MathUtils::calculateTriangleNormal(const glm::vec3 triangleVertices[3])
{
    return glm::normalize(
        glm::cross(
            triangleVertices[2] - triangleVertices[0],
            triangleVertices[1] - triangleVertices[0]
        )
    );
}

std::vector<std::array<uint64_t, 3>> MathUtils::triangulatePolygon(const std::vector<uint32_t> &polygonVertices, const std::vector<glm::vec3>& positions)
{
    using Point3 = CGAL::Simple_cartesian<float>::Point_3;
	std::vector<Point3> cgalPolygonVertices;
	std::vector<std::array<uint64_t, 3>> cgalTriangleIndices;
	std::vector<uint32_t> triangleIndices;
	for (const uint32_t vertex : polygonVertices)
		cgalPolygonVertices.emplace_back(positions[vertex].x, positions[vertex].y, positions[vertex].z);

	CGAL::advancing_front_surface_reconstruction(
		cgalPolygonVertices.begin(),
		cgalPolygonVertices.end(),
		std::back_inserter(cgalTriangleIndices),
        10.0, 
        0.8
	);
	
	return cgalTriangleIndices;
}

bool MathUtils::isPointInsideTriangle(const glm::vec3& projectedPoint, const glm::vec3 trianglePoints[3])
{
    glm::vec3 planeNormal = glm::cross(trianglePoints[2] - trianglePoints[0], trianglePoints[1] - trianglePoints[0]);
    glm::vec3 edge, pointToTrianglePoint, triangleNormal;

    // Edge 1
    edge = trianglePoints[0] - trianglePoints[1];
    pointToTrianglePoint = projectedPoint - trianglePoints[1];
    triangleNormal = glm::cross(edge, pointToTrianglePoint);
    if (glm::dot(planeNormal, triangleNormal) < 0) return false;

    // Edge 2
    edge = trianglePoints[1] - trianglePoints[2];
    pointToTrianglePoint = projectedPoint - trianglePoints[2];
    triangleNormal = glm::cross(edge, pointToTrianglePoint);
    if (glm::dot(planeNormal, triangleNormal) < 0) return false;

    // Edge 3
    edge = trianglePoints[2] - trianglePoints[0];
    pointToTrianglePoint = projectedPoint - trianglePoints[0];
    triangleNormal = glm::cross(edge, pointToTrianglePoint);
    if (glm::dot(planeNormal, triangleNormal) < 0) return false;

    return true;
}
