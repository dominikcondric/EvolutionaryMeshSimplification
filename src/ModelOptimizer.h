#pragma once
#include "Utility/Model.h"

class ModelOptimizer {
private:
	struct Solution {
		std::vector<uint32_t> bitstring;
		std::vector<uint32_t> newIndices;
		float error = 0.f;
	};

public:
	ModelOptimizer(uint32_t _generationCount, uint32_t _populationCount,
		uint32_t _offspringCount, float _mutationProbability, uint32_t _tournamentParticipantsCount);
	~ModelOptimizer() = default;
	void optimize(Cala::Model* model);

	void setGenerationCount(uint32_t _generationCount) { generationCount = _generationCount; }
	void setOffsprintCount(uint32_t _offspringCount) { offspringCount = _offspringCount; }
	void setMutationProbability(float _mutationProbability) { mutationProbability = glm::clamp(_mutationProbability, 0.f, 1.f); }
	void setPopulationCount(uint32_t _populationCount) { populationCount = _populationCount; }
	void setTournamentParticipantsCount(uint32_t _tournamentParticipantsCount) { tournamentParticipantsCount = _tournamentParticipantsCount; }

	uint32_t getGenerationCount() const { return generationCount; }
	uint32_t getOffspringCount() const { return offspringCount; }
	uint32_t getPopulationCount() const { return populationCount; }
	uint32_t getTournamentParticipantsCount() const { return tournamentParticipantsCount; }
	float getMutationProbability() const { return mutationProbability; }


private:
	uint32_t generationCount;
	float mutationProbability;
	uint32_t populationCount;
	uint32_t offspringCount;
	uint32_t tournamentParticipantsCount;

	void regenerateModel(Cala::Model* model, const std::vector<uint32_t>& bitstring, std::vector<uint32_t>& indices) const;
	std::vector<Solution> generateInitialPopulation(const Cala::Model* model, uint32_t populationCount) const;
	ModelOptimizer::Solution generateGenotype(const Cala::Model* model) const;
	void generateNewIndicesAndError(const Cala::Model* model, Solution& solution) const;
	void generateOffspring(const Cala::Model* model, std::vector<Solution>& population) const;
	uint32_t selectParent(const std::vector<Solution>& population) const;
	float calculateSquareError(const std::vector<float>& modelVertexData, uint32_t oldVertexIndex, const uint32_t newTriangleIndices[3]) const;
	void mutate(std::vector<uint32_t>& bitstring) const;
};
