#include "ModelOptimizer.h"
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <glm/gtx/string_cast.hpp>
#include <set>
#include <random>
#include <glm/gtc/random.hpp>

ModelOptimizer::ModelOptimizer(uint32_t _generationCount, uint32_t _populationCount, 
	uint32_t _offspringCount, float _mutationProbability, uint32_t _tournamentParticipantsCount) :
	generationCount(_generationCount), populationCount(_populationCount), offspringCount(_offspringCount),
	mutationProbability(glm::clamp(_mutationProbability, 0.f, 1.f)), tournamentParticipantsCount(_tournamentParticipantsCount)
{
}

void ModelOptimizer::optimize(Cala::Model* model)
{
	model->removeReduntantVertices();
	std::cout << "Optimizing model..." << '\n';
	std::vector<Solution> population = generateInitialPopulation(model, populationCount);
	auto predicate = [](const Solution& s1, const Solution& s2) {
		return s1.error < s2.error;
	};

	for (uint32_t i = 0; i < generationCount; ++i)
	{
		std::cout << "\tGeneration " << i+1 << ": " << '\n';
		generateOffspring(model, population);
		std::sort(population.begin(), population.end(), predicate);
		population.erase(population.begin() + populationCount, population.end());
		std::cout << "\n\t\tBest error: " << population.front().error << "\n\n";

		/*if (population.front().error < 10.f)
		{
			std::cout << "\tInsignificant error...breaking!" << '\n';
			break;
		}*/
	}

	std::cout << std::endl;
	Solution best = population.front();
	regenerateModel(model, best.bitstring, best.newIndices);
}

void ModelOptimizer::regenerateModel(Cala::Model* model, const std::vector<uint32_t>& bitstring, std::vector<uint32_t>& indices) const
{
	std::vector<float> newVertexData;
	std::vector<uint32_t> oldToNewIndexMapping(model->getVertexCount());
	for (uint32_t i = 0; i < model->getVertexCount(); ++i)
	{
		if (bitstring[i] == 1)
		{
			oldToNewIndexMapping[i] = (uint32_t)newVertexData.size() / 8;
			for (int j = 0; j < 8; ++j)
				newVertexData.push_back(model->getVertexData()[i * 8 + j]);
		}
	}

	for (auto& index : indices)
	{
		index = oldToNewIndexMapping[index];
	}

	std::cout << "\tPercentage of original vertices: " << newVertexData.size() * 100.f / model->getVertexData().size() << '\n' << std::endl;
	model->loadCustomModel(newVertexData, (uint32_t)newVertexData.size() / 8, indices, model->getLayoutSpecification(), model->getDrawingMode());
}

std::vector<ModelOptimizer::Solution> ModelOptimizer::generateInitialPopulation(const Cala::Model* model, uint32_t populationCount) const
{
	std::vector<Solution> population;
	population.reserve(populationCount);
	for (uint32_t i = 0; i < populationCount; ++i)
	{
		population.emplace_back(generateGenotype(model));
		std::cout << "\tGenerating solution " << i+1 << '\n';
	}

	return population;
}

ModelOptimizer::Solution ModelOptimizer::generateGenotype(const Cala::Model* model) const
{
	Solution solution;
	solution.bitstring.reserve(model->getVertexCount());

	for (uint32_t i = 0; i < model->getVertexCount(); ++i)
		solution.bitstring.push_back(1);

	uint32_t nrOfVerticesToRemove = glm::linearRand<uint32_t>((uint32_t)solution.bitstring.size() / 3, (uint32_t)solution.bitstring.size() / 2);

	for (uint32_t i = 0; i < nrOfVerticesToRemove; ++i)
	{
		solution.bitstring[glm::linearRand<uint32_t>(0, (uint32_t)solution.bitstring.size() - 1)] = 0;
	}

	generateNewIndicesAndError(model, solution);
	return solution;
}

void ModelOptimizer::generateNewIndicesAndError(const Cala::Model* model, Solution& solution) const
{
	const std::vector<uint32_t>& modelIndexData = model->getIndexData();
	const std::vector<float>& modelVertexData = model->getVertexData();
	std::vector<std::set<uint32_t>> neighboursTable(model->getVertexCount(), std::set<uint32_t>()); // table of neighbours
	std::vector<std::set<uint32_t>> indexMap(model->getVertexCount(), std::set<uint32_t>());  // index of each vertex in indices array
	std::set<uint32_t> indicesToDelete;
	std::vector<std::set<uint32_t>> triangleIndicesPerVertex(model->getVertexCount());
	solution.newIndices = modelIndexData;

	for (uint32_t i = 0; i < model->getIndexCount() - 2; i += 3)
	{
		neighboursTable[modelIndexData[i]].insert(modelIndexData[i + 1]);
		neighboursTable[modelIndexData[i]].insert(modelIndexData[i + 2]);
		neighboursTable[modelIndexData[i + 1]].insert(modelIndexData[i]);
		neighboursTable[modelIndexData[i + 1]].insert(modelIndexData[i + 2]);
		neighboursTable[modelIndexData[i + 2]].insert(modelIndexData[i]);
		neighboursTable[modelIndexData[i + 2]].insert(modelIndexData[i + 1]);

		indexMap[modelIndexData[i]].insert(i);
		indexMap[modelIndexData[i + 1]].insert(i + 1);
		indexMap[modelIndexData[i + 2]].insert(i + 2);

		triangleIndicesPerVertex[modelIndexData[i]].insert(i);
		triangleIndicesPerVertex[modelIndexData[i + 1]].insert(i);
		triangleIndicesPerVertex[modelIndexData[i + 2]].insert(i);
	}

	for (uint32_t i = 0; i < (uint32_t)solution.bitstring.size(); ++i)
	{
		if (solution.bitstring[i] == 0 && !neighboursTable[i].empty())
		{
			/*
			* For every vertex that should be removed do:
			*	- Pick a new pivot vertex from its neighbour table
			*	- For every neighbour of removing vertex remove removing vertex index
			*	- For every neighbour that is not a pivot, add pivot as a new neighbour
			*	- For pivot add all other neighbours as new neighbour
			*/
			auto& neighboursList = neighboursTable[i];

			/*int pivot = *neighboursList.begin();
			for (const int neighbour : neighboursList)
			{
				if (neighboursTable[neighbour].size() < neighboursTable[pivot].size())
					pivot = neighbour;
			}*/

			int pivot = -1;
			float smallestDistance = (float)INT_MAX;
			glm::vec3 oldVertexPosition(
				modelVertexData[i * 8],
				modelVertexData[i * 8 + 1],
				modelVertexData[i * 8 + 2]
			);

			for (const int neighbour : neighboursList)
			{
				glm::vec3 neighbourVertexPosition(
					modelVertexData[neighbour * 8],
					modelVertexData[neighbour * 8 + 1],
					modelVertexData[neighbour * 8 + 2]
				);

				float dist = glm::distance(oldVertexPosition, neighbourVertexPosition);
				if (dist < smallestDistance)
				{
					pivot = neighbour;
					smallestDistance = dist;
				}
			}

			for (const auto neighbour : neighboursList)
			{
				neighboursTable[neighbour].erase(i);

				if (neighbour != pivot)
				{
					for (const uint32_t triangleIndex : triangleIndicesPerVertex[i])
						triangleIndicesPerVertex[pivot].insert(triangleIndex);

					neighboursTable[neighbour].insert(pivot);
					neighboursTable[pivot].insert(neighbour);
				}
			}

			/*
			* Find the first index of a triangle containing removing vertex
			* Replace removing vertex index from indices list with new pivot vertex index
			* If the triangle already contains the new pivot vertex, add it to list of triangles to remove and
			  remove trinagles vertex indices from indexMap
			*/
			for (const uint32_t index : indexMap[i])
			{
				solution.newIndices[index] = pivot;
				indexMap[pivot].insert(index);
			}

			indexMap[i].clear(); // Delete all indices of a vertex-to-remove
		}
	}

	// Calculating square errors
	for (uint32_t i = 0; i < (uint32_t)solution.bitstring.size(); ++i)
	{
		if (solution.bitstring[i] == 0)
		{
			for (const uint32_t triangleIndex : triangleIndicesPerVertex[i])
			{
				uint32_t triangleVertexIndices[3] = {
					solution.newIndices[triangleIndex],
					solution.newIndices[triangleIndex + 1],
					solution.newIndices[triangleIndex + 2]
				};

				float error = calculateSquareError(model->getVertexData(), i, triangleVertexIndices);
				if (error == -1.f)
				{
					indicesToDelete.insert(triangleIndex);
				}
				else
				{
					solution.error += error;
				}
			}
		}
	}

	/*
	* Remove trinagles which decomposed to a line because of two vertices being the same (pivot) vertex
	* Sorting is needed to ensure removal from highest to lowest index triangles
	*/
	for (auto iterator = indicesToDelete.rbegin(); iterator != indicesToDelete.rend(); iterator++)
	{
		auto first = solution.newIndices.begin() + *iterator;
		solution.newIndices.erase(first, first + 3);
	}
}

void ModelOptimizer::generateOffspring(const Cala::Model* model, std::vector<Solution>& population) const
{
	std::vector<Solution> offspring;
	for (uint32_t i = 0; i < offspringCount; ++i)
	{
		std::cout << "\t\tGenerating offspring " << i + 1 << "...\n";
		Solution solution;
		uint32_t firstParentIndex = selectParent(population);
		uint32_t secondParentIndex = firstParentIndex;
		while (secondParentIndex == firstParentIndex)
		{
			secondParentIndex = selectParent(population);
		}

		const std::vector<uint32_t>& firstParentBitstring = population[firstParentIndex].bitstring;
		const std::vector<uint32_t>& secondParentBitstring = population[secondParentIndex].bitstring;
		solution.bitstring = firstParentBitstring;

		for (uint32_t j = 0; j < (uint32_t)solution.bitstring.size(); ++j)
		{
			if (glm::linearRand(0.f, 1.f) < 0.5f)
				solution.bitstring[j] = secondParentBitstring[j];
		}

		if (glm::linearRand(0.f, 1.f) < mutationProbability)
		{
			mutate(solution.bitstring);
		}

		generateNewIndicesAndError(model, solution);
		offspring.push_back(solution);
	}

	population.insert(population.end(), offspring.begin(), offspring.end());
}

uint32_t ModelOptimizer::selectParent(const std::vector<Solution>& population) const
{
	uint32_t bestIndex = glm::linearRand<uint32_t>(0, (uint32_t)population.size() - 1);
	const Solution* best = &population[bestIndex];

	for (uint32_t i = 0; i < tournamentParticipantsCount - 1; ++i)
	{
		uint32_t randomIndex = glm::linearRand<uint32_t>(0, (uint32_t)population.size() - 1);
		const Solution* participant = &population[randomIndex];

		if (participant->error < best->error)
		{
			best = participant;
			bestIndex = randomIndex;
		}
	}

	return bestIndex;
}

/*
* Arguments:
*	oldVertexIndex - index of a vertex 
*	newTriangleIndices - indices of the index of the first vertex in the triangle from the index data
*	updatedIndices - updated index list
*/
float ModelOptimizer::calculateSquareError(const std::vector<float>& modelVertexData, uint32_t oldVertexIndex, const uint32_t newTriangleIndices[3]) const
{
	glm::vec3 oldVertexPosition(modelVertexData[oldVertexIndex * 8], modelVertexData[oldVertexIndex * 8 + 1], modelVertexData[oldVertexIndex * 8 + 2]);
	glm::vec3 triangleVertexPositions[3];
	for (int i = 0; i < 3; ++i)
	{
		triangleVertexPositions[i] = glm::vec3(
			modelVertexData[newTriangleIndices[i] * 8],
			modelVertexData[newTriangleIndices[i] * 8 + 1],
			modelVertexData[newTriangleIndices[i] * 8 + 2]
		);
	}

	glm::vec3 trianglePlaneNormal = glm::normalize(
		glm::cross(triangleVertexPositions[2] - triangleVertexPositions[0], triangleVertexPositions[1] - triangleVertexPositions[0])
	);

	if (glm::isnan(trianglePlaneNormal.x))
	{
		return -1.f;
	}
	else
	{
		glm::vec3 anchorToVertexVector = oldVertexPosition - triangleVertexPositions[0];
		float planeToVertexDistance = glm::dot(anchorToVertexVector, trianglePlaneNormal);

		glm::vec3 triangleCentroid = (triangleVertexPositions[0] + triangleVertexPositions[1] + triangleVertexPositions[2]) / 3.f;
		return (planeToVertexDistance * planeToVertexDistance) * glm::distance(oldVertexPosition, triangleCentroid);
	}
}

void ModelOptimizer::mutate(std::vector<uint32_t>& bitstring) const
{
	auto& value = bitstring[glm::linearRand<uint32_t>(0, (uint32_t)bitstring.size() - 1)];
	if (value == 0)
		value = 1;
	else
		value = 0;
}