#pragma once
#include "Utility/Model.h"
#include <filesystem>

class ModelLoader {
public:
	static Cala::Model loadOBJModel(const std::filesystem::path& modelPath);
	static std::vector<Cala::Model> loadOBJModels(const std::filesystem::path& modelPath);
};