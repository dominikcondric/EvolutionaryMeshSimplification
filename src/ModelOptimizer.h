#pragma once
#include "Cala/Utility/Model.h"
#include <unordered_set>
#include <set>

class ModelOptimizer {
private:
	struct Solution {
		std::unordered_set<uint32_t> verticesToRemove;
		std::vector<uint32_t> newIndices;
		float error = 0.f;
		uint32_t rank;
		float distance;

		void reset()
		{
			rank = -1;
			distance = 0.f;
		}
	};

	template<typename T> using TwoDimVector = std::vector<std::vector<T>>;

public:
	ModelOptimizer() = default;
	~ModelOptimizer() = default;
    void optimize(Cala::Model *model, uint32_t _generationCount, uint32_t _populationCount, uint32_t _offspringCount, float _mutationProbability, uint32_t _tournamentParticipantsCount);

private:
	uint32_t generationCount;
	float mutationProbability;
	uint32_t populationCount;
	uint32_t offspringCount;
	uint32_t tournamentParticipantsCount;


    void regenerateModel(Cala::Model *model, Solution& solution) const;
    std::vector<Solution> generateInitialPopulation(const Cala::Model* model, uint32_t populationCount) const;
	ModelOptimizer::Solution generateGenotype(const Cala::Model* model) const;
	void generateIndicesAndError(const Cala::Model* model, Solution& solution) const;
    void generateOffspring(const Cala::Model *model, std::vector<Solution> &population) const;
    uint32_t selectParent(const std::vector<Solution>& population) const;
	float calculateSquareError(const glm::vec3& vertexPosition, const glm::vec3 trianglePositions[3]) const;
	void mutate(uint32_t vertexCount, std::unordered_set<uint32_t>& verticesToRemove) const;
    std::vector<std::vector<uint32_t>> performNonDominatedSort(std::vector<Solution> &population);
    void calculateCrowdingDistance(const std::vector<uint32_t>& paretoFronts, std::vector<Solution>& population);
    std::vector<Solution> nextGeneration(const std::vector<Solution> &population, const std::vector<std::vector<uint32_t>> &paretoFronts);
    void printParetoFronts(const std::vector<std::vector<uint32_t>>& paretoFronts, const std::vector<Solution>& population) const;
	void createPolygonRecursively(const std::vector<std::set<uint32_t>>& neighboursTable, std::unordered_set<uint32_t>& verticesToRemove, uint32_t polygonVertex, std::unordered_set<uint32_t>& removedVerticesPerPolygon, std::set<uint32_t>& polygonSet) const;
	void plotParetoFronts(const TwoDimVector<uint32_t> &paretoFronts, const std::vector<Solution>& population, uint32_t originalVertexCount) const;
};
