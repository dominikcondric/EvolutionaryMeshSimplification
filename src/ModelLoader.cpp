#include "ModelLoader.h"
#include "OBJ_Loader.h"

#undef OBJL_CONSOLE_OUTPUT 

Cala::Model ModelLoader::loadOBJModel(const std::filesystem::path& modelPath)
{
	objl::Loader loader;
	loader.LoadFile(modelPath.string());
	Cala::Model model;

	std::vector<float> vertexData;
	for (const auto& vertex : loader.LoadedVertices)
	{
		// Positions
		vertexData.push_back(vertex.Position.X);
		vertexData.push_back(vertex.Position.Y);
		vertexData.push_back(vertex.Position.Z);

		// Normals
		vertexData.push_back(vertex.Normal.X);
		vertexData.push_back(vertex.Normal.Y);
		vertexData.push_back(vertex.Normal.Z);

		// Tex coords
		vertexData.push_back(vertex.TextureCoordinate.X);
		vertexData.push_back(1.f - vertex.TextureCoordinate.Y);
	}
		
	std::vector<Cala::Model::VertexLayoutSpecification> layoutSpecification = {
		Cala::Model::VertexLayoutSpecification{ 0, 3, 8 * sizeof(float), 0, 0 },
		Cala::Model::VertexLayoutSpecification{ 1, 3, 8 * sizeof(float), sizeof(glm::vec3), 0 },
		Cala::Model::VertexLayoutSpecification{ 2, 2, 8 * sizeof(float), 2 * sizeof(glm::vec3), 0 }
	};

	model.loadCustomModel(vertexData, loader.LoadedVertices.size(), loader.LoadedIndices, layoutSpecification, Cala::Model::DrawingMode::Triangles);
	return model;
}

std::vector<Cala::Model> ModelLoader::loadOBJModels(const std::filesystem::path& modelPath)
{
	std::vector<Cala::Model> models;
	objl::Loader loader;
	loader.LoadFile(modelPath.string());
	for (const objl::Mesh& mesh : loader.LoadedMeshes)
	{
		std::vector<float> vertexData;
		for (const auto& vertex : mesh.Vertices)
		{
			// Positions
			vertexData.push_back(vertex.Position.X);
			vertexData.push_back(vertex.Position.Y);
			vertexData.push_back(vertex.Position.Z);

			// Normals
			vertexData.push_back(vertex.Normal.X);
			vertexData.push_back(vertex.Normal.Y);
			vertexData.push_back(vertex.Normal.Z);

			// Tex coords
			vertexData.push_back(vertex.TextureCoordinate.X);
			vertexData.push_back(1.f - vertex.TextureCoordinate.Y);
		}

		std::vector<Cala::Model::VertexLayoutSpecification> layoutSpecification = {
			Cala::Model::VertexLayoutSpecification{ 0, 3, 8 * sizeof(float), 0, 0 },
			Cala::Model::VertexLayoutSpecification{ 1, 3, 8 * sizeof(float), sizeof(glm::vec3), 0 },
			Cala::Model::VertexLayoutSpecification{ 2, 2, 8 * sizeof(float), 2 * sizeof(glm::vec3), 0 }
		};

		models.push_back(Cala::Model().loadCustomModel(vertexData, mesh.Vertices.size(), mesh.Indices, layoutSpecification, Cala::Model::DrawingMode::Triangles));
	}

	return models;
}
