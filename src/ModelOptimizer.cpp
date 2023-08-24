#include "ModelOptimizer.h"
#include <unordered_map>
#include <set>
#include <stack>
#include <algorithm>
#include <iostream>
#include <random>
#include <glm/gtc/random.hpp>
#include <glm/gtc/constants.hpp>
#include "MathUtils.h"
#include <matplotlibcpp.h>
#include <mutex>

int generateRandomIntInRange(int min = 0, int max = INT_MAX)
{
	static thread_local std::mt19937 rng(std::hash<std::thread::id>{}(std::this_thread::get_id()));
	std::uniform_int_distribution<uint32_t> dist(min, max);
	int random = dist(rng);
	return random;
}

float generateRandomFloat()
{
	return generateRandomIntInRange() / (float)INT_MAX;
}

void ModelOptimizer::optimize(Cala::Model* model, uint32_t _generationCount, uint32_t _populationCount,
		uint32_t _offspringCount, float _mutationProbability, uint32_t _tournamentParticipantsCount)
{
	generationCount = _generationCount;
	populationCount = _populationCount;
	offspringCount = _offspringCount;
	mutationProbability = _mutationProbability;
	tournamentParticipantsCount = _tournamentParticipantsCount;

	std::cout << "Optimizing model: " << model->getModelName();
	std::vector<Solution> population = generateInitialPopulation(model, populationCount);

	TwoDimVector<uint32_t> paretoFronts = performNonDominatedSort(population);
	for (auto& front : paretoFronts)
	{
		calculateCrowdingDistance(front, population);
		std::sort(front.begin(), front.end(), [&population, &front](uint32_t first, uint32_t second) {
			return population[first].distance > population[second].distance;
		});
	}

	for (uint32_t i = 0; i < generationCount; ++i)
	{
		std::cout << "\n\tGeneration " << i+1 << ": " << '\n';
		std::vector<Solution> offspring = generateOffspring(model, population);
		population.insert(population.end(), offspring.begin(), offspring.end());

		for (Solution& s : population)
			s.reset();

		paretoFronts = performNonDominatedSort(population);
		for (auto& front : paretoFronts)
		{
			calculateCrowdingDistance(front, population);
			std::sort(front.begin(), front.end(), [&population, &front](uint32_t first, uint32_t second) {
				return population[first].distance > population[second].distance;
			});
		}

		population =  nextGeneration(population, paretoFronts);
	}

	const uint32_t vertexCount = model->getPositions().size();
	plotParetoFronts(paretoFronts, population, vertexCount);

	std::cout << "\n1. pareto front: \n";
	for (int i = 0; i < paretoFronts[0].size(); ++i)
	{
		const Solution& s = population[paretoFronts[0][i]];
		std::cout << "\t\t" << i << " - error: " << s.error << 
			", vertices kept: " << (vertexCount - s.verticesToRemove.size()) * 100.f / vertexCount << "%\n";
	}

	uint32_t choice;
	do {
		std::cout << "\n\t\tChoose a solution from 1. front (e.g. 0, 1, 4, 6...): ";
		std::cin >> choice;
	} while (choice >= paretoFronts[0].size());

	regenerateModel(model, population[paretoFronts[0][choice]]);
	std::cout << "\n\tPercentage of original vertices: " << model->getPositions().size() * 100.f / (float)vertexCount << '\n' << std::endl;
}

void ModelOptimizer::regenerateModel(Cala::Model* model, Solution& solution) const
{
	std::vector<glm::vec3> positions;
	std::vector<glm::vec3> normals;
	std::vector<glm::vec2> textureCoordinates;
	const uint32_t vertexCount = model->getPositions().size();
	std::vector<uint32_t> oldToNewIndexMapping(vertexCount);
	for (uint32_t i = 0; i < vertexCount; ++i)
	{
		if (solution.verticesToRemove.find(i) == solution.verticesToRemove.end())
		{
			oldToNewIndexMapping[i] = (uint32_t)positions.size();
			positions.push_back(model->getPositions()[i]);
			if (!model->getNormals().empty()) normals.push_back(model->getNormals()[i]);
			if (!model->getTextureCoordinates().empty()) textureCoordinates.push_back(model->getTextureCoordinates()[i]);
		}
	}

	for (auto& index : solution.newIndices)
	{
		index = oldToNewIndexMapping[index];
	}

	model->loadCustomModel(positions, normals, textureCoordinates, solution.newIndices, model->getDrawingMode(), "", "");
}

std::vector<ModelOptimizer::Solution> ModelOptimizer::generateInitialPopulation(const Cala::Model* model, uint32_t populationCount) const
{
	std::vector<Solution> population;
	std::vector<std::future<Solution>> futures;
	population.reserve(populationCount);
	for (uint32_t i = 0; i < populationCount; ++i)
		futures.push_back(std::async(std::launch::async, generateGenotype, this, model));

	for (uint32_t i = 0; i < populationCount; ++i)
	{
		population.push_back(futures[i].get());
		std::cout << "\n\tGenerating solution " << i+1;
	}

	return population;
}

ModelOptimizer::Solution ModelOptimizer::generateGenotype(const Cala::Model* model) const
{
	uint32_t vertexCount = model->getPositions().size();
	Solution solution;

	float threshold = generateRandomFloat();
	threshold = 0.1f + 0.9f * threshold;
	for (int i = 0; i < vertexCount; ++i)
	{
		if (generateRandomFloat() > threshold)
			solution.verticesToRemove.insert(i);
	}

	generateIndicesAndError(model, solution);
	return solution;
}

void ModelOptimizer::createPolygon(const std::vector<std::set<uint32_t>>& neighboursTable, std::unordered_set<uint32_t>& verticesToRemove, uint32_t vertexToRemove, std::unordered_set<uint32_t>& removedVerticesPerPolygon, std::set<uint32_t>& polygonSet) const
{
	std::stack<uint32_t> verticesToProcess({ vertexToRemove });
	while (!verticesToProcess.empty())
	{
		vertexToRemove = verticesToProcess.top();
		verticesToProcess.pop();
		removedVerticesPerPolygon.insert(vertexToRemove);
		verticesToRemove.erase(vertexToRemove);
		for (const uint32_t neighbour : neighboursTable[vertexToRemove])
		{
			if (removedVerticesPerPolygon.find(neighbour) == removedVerticesPerPolygon.end()) // If vertex is not added in removed vertices for current polygon
			{
				if (verticesToRemove.find(neighbour) == verticesToRemove.end()) // If vertex is not in removed vertices list
					polygonSet.insert(neighbour);
				else
				{
					verticesToProcess.push(neighbour);
				}
			}
		}
	}
}

void ModelOptimizer::generateIndicesAndError(const Cala::Model* model, Solution& solution) const
{
	const uint32_t vertexCount = model->getPositions().size();
	const std::vector<uint32_t>& modelIndices = model->getIndices();
	const std::vector<glm::vec3>& modelPositions = model->getPositions();
	std::vector<std::set<uint32_t>> neighboursTable(vertexCount, std::set<uint32_t>()); // table of neighbours
	std::unordered_map<uint32_t, float> errors;

	for (uint32_t i = 0; i < modelIndices.size() - 2; i += 3)
	{
		neighboursTable[modelIndices[i]].insert(modelIndices[i + 1]);
		neighboursTable[modelIndices[i]].insert(modelIndices[i + 2]);
		neighboursTable[modelIndices[i + 1]].insert(modelIndices[i]);
		neighboursTable[modelIndices[i + 1]].insert(modelIndices[i + 2]);
		neighboursTable[modelIndices[i + 2]].insert(modelIndices[i]);
		neighboursTable[modelIndices[i + 2]].insert(modelIndices[i + 1]);

		if (solution.verticesToRemove.find(modelIndices[i]) == solution.verticesToRemove.end() &&
			solution.verticesToRemove.find(modelIndices[i + 1]) == solution.verticesToRemove.end() &&
			solution.verticesToRemove.find(modelIndices[i + 2]) == solution.verticesToRemove.end())
		{
			solution.newIndices.push_back(modelIndices[i]);
			solution.newIndices.push_back(modelIndices[i + 1]);
			solution.newIndices.push_back(modelIndices[i + 2]);
		}
	}

	std::unordered_set<uint32_t> verticesToRemoveCopy(solution.verticesToRemove);
	while (!verticesToRemoveCopy.empty())
	{
		uint32_t vertexToRemove = *verticesToRemoveCopy.begin();
		std::unordered_set<uint32_t> verticesToRemovePerPolygon;
		std::set<uint32_t> polygonSet;
		createPolygon(neighboursTable, verticesToRemoveCopy, vertexToRemove, verticesToRemovePerPolygon, polygonSet);

		std::vector<uint32_t> polygonVerticesArray(polygonSet.begin(), polygonSet.end());
		std::vector<std::array<uint64_t, 3>> newTriangles = MathUtils::triangulatePolygon(polygonVerticesArray, modelPositions);
		for (const auto& newTriangle : newTriangles)
		{
			uint32_t newTriangleIndices[3] {
				polygonVerticesArray[newTriangle[0]],
				polygonVerticesArray[newTriangle[1]],
				polygonVerticesArray[newTriangle[2]]
			};

			glm::vec3 newTrianglePoints[3] = {
				modelPositions[newTriangleIndices[0]],
				modelPositions[newTriangleIndices[1]],
				modelPositions[newTriangleIndices[2]]
			};

			float orientation = glm::dot(
				glm::cross(
					newTrianglePoints[2] - newTrianglePoints[0],
					newTrianglePoints[1] - newTrianglePoints[0]
				),
				model->getNormals()[newTriangleIndices[0]]
			);

			if (orientation > 0.f)
			{
				std::swap(newTriangleIndices[1], newTriangleIndices[2]);
				newTrianglePoints[1] = modelPositions[newTriangleIndices[1]];
				newTrianglePoints[2] = modelPositions[newTriangleIndices[2]];
			}

			for (const uint32_t vertexToRemove : verticesToRemovePerPolygon)
			{
				float error = calculateSquareError(modelPositions[vertexToRemove], newTrianglePoints);
				if (error >= 0.f && (errors.find(vertexToRemove) == errors.end() || error < errors[vertexToRemove]))
					errors[vertexToRemove] = error;
			}

			solution.newIndices.push_back(newTriangleIndices[0]);
			solution.newIndices.push_back(newTriangleIndices[1]);
			solution.newIndices.push_back(newTriangleIndices[2]);
		}
	}

	solution.error = std::accumulate(errors.begin(), errors.end(), 0.f, [](float res, const std::pair<uint32_t, float>& el) {
		return res + el.second;
	});
}

std::vector<ModelOptimizer::Solution> ModelOptimizer::generateOffspring(const Cala::Model* model, const std::vector<Solution>& population) const
{
	std::vector<std::future<Solution>> futures;
	for (uint32_t i = 0; i < offspringCount; ++i)
	{
		futures.push_back(std::async(std::launch::async, generateChild, this, model, population));
	}

	std::vector<Solution> offspring;
	for (uint32_t i = 0; i < offspringCount; ++i)
	{
		offspring.push_back(futures[i].get());
		std::cout << "\t\tGenerating offspring " << i + 1 << "...\n";
	}

	return offspring;
}

ModelOptimizer::Solution ModelOptimizer::generateChild(const Cala::Model* model, const std::vector<Solution>& population) const
{
	Solution solution;
	uint32_t firstParentIndex = selectParent(population);
	uint32_t secondParentIndex = firstParentIndex;
	while (secondParentIndex == firstParentIndex)
		secondParentIndex = selectParent(population);

	const std::unordered_set<uint32_t>& firstParentBitstring = population[firstParentIndex].verticesToRemove;
	const std::unordered_set<uint32_t>& secondParentBitstring = population[secondParentIndex].verticesToRemove;

	for (uint32_t j = 0; j < model->getPositions().size(); ++j)
	{
		float randomFloat = generateRandomFloat();
		if ((randomFloat < 0.5f && firstParentBitstring.find(j) != solution.verticesToRemove.end())
			|| (randomFloat > 0.5f && secondParentBitstring.find(j) != solution.verticesToRemove.end()))
			solution.verticesToRemove.insert(j);
	}


	float randomFloat = generateRandomFloat();
	if (randomFloat < mutationProbability)
	{
		mutate(model->getPositions().size(), solution.verticesToRemove);
	}

	generateIndicesAndError(model, solution);

	return solution;
}

uint32_t ModelOptimizer::selectParent(const std::vector<Solution>& population) const
{
	uint32_t bestIndex = generateRandomIntInRange(0, (uint32_t)population.size() - 1);
	const Solution* best = &population[bestIndex];

	for (uint32_t i = 0; i < tournamentParticipantsCount - 1; ++i)
	{
		uint32_t randomIndex = generateRandomIntInRange(0, (uint32_t)population.size() - 1);
		const Solution* participant = &population[randomIndex];

		if (participant->rank < best->rank || (participant->rank == best->rank && participant->distance > best->distance))
		{
			best = participant;
			bestIndex = randomIndex;
		}
	}

	return bestIndex;
}

float ModelOptimizer::calculateSquareError(const glm::vec3& vertexPosition, const glm::vec3 trianglePoints[3]) const
{
	const glm::vec3 trianglePlaneNormal = MathUtils::calculateTriangleNormal(trianglePoints);
	const glm::vec3 anchorToVertexVector = vertexPosition - trianglePoints[0];
	const float planeToVertexDistance = glm::dot(anchorToVertexVector, trianglePlaneNormal);
	const glm::vec3 projectedPoint = vertexPosition - planeToVertexDistance * trianglePlaneNormal;

	if (MathUtils::isPointInsideTriangle(projectedPoint, trianglePoints))
	{
		return glm::abs(planeToVertexDistance);
	}
	else
	{
		return -1.f;
	}
}

void ModelOptimizer::mutate(uint32_t vertexCount, std::unordered_set<uint32_t>& verticesToRemove) const
{
	uint32_t randomIndex = glm::linearRand(0U, vertexCount - 1);
	if (verticesToRemove.find(randomIndex) != verticesToRemove.end())
		verticesToRemove.insert(randomIndex);
	else
		verticesToRemove.erase(randomIndex);
}

ModelOptimizer::TwoDimVector<uint32_t> ModelOptimizer::performNonDominatedSort(std::vector<Solution>& population)
{
	TwoDimVector<uint32_t> paretoFronts(1);
	TwoDimVector<uint32_t> listOfDominatedIndices(population.size(), std::vector<uint32_t>());
	std::vector<uint32_t> dominationCount(population.size(), 0);

	for (int i = 0; i < population.size(); ++i)
	{
		for (int j = 0; j < population.size(); ++j)
		{
			if (i == j)
				continue;

			bool isDominating = false;
			bool isDominated = false;

			if (population[i].error < population[j].error)
				isDominating = true;
			else if (population[i].error > population[j].error)
				isDominated = true;

			if (population[i].verticesToRemove.size() > population[j].verticesToRemove.size())
				isDominating = true;
			else if (population[i].verticesToRemove.size() < population[j].verticesToRemove.size())
				isDominated = true;

			if (isDominating && !isDominated)
				listOfDominatedIndices[i].push_back(j);
			else if (!isDominating && isDominated)
				dominationCount[i]++;
		}

		if (dominationCount[i] == 0)
		{
			population[i].rank = 0;
			paretoFronts[0].push_back(i);
		}
	}

	int i = 0;
	while (true)
	{
		std::vector<uint32_t> nextParetoFront;
		for (uint32_t j : paretoFronts[i])
		{
			for (uint32_t k : listOfDominatedIndices[j])
			{
				dominationCount[k]--;
				if (dominationCount[k] == 0)
				{
					population[k].rank = i + 1;
					nextParetoFront.push_back(k);
				}
			}
		}
		
		i++;
		if (!nextParetoFront.empty())
			paretoFronts.push_back(nextParetoFront);
		else
			break;
	}

	return paretoFronts;
}

void ModelOptimizer::calculateCrowdingDistance(const std::vector<uint32_t> &paretoFront, std::vector<Solution> &population)
{
	std::vector<uint32_t> sortedFrontVertexCount = paretoFront;
	std::vector<uint32_t> sortedFrontError = paretoFront;

	std::sort(sortedFrontError.begin(), sortedFrontError.end(), [&](uint32_t first, uint32_t second){
		return population[first].error < population[second].error;
	});

	std::sort(sortedFrontVertexCount.begin(), sortedFrontVertexCount.end(), [&](uint32_t first, uint32_t second){
		return population[first].verticesToRemove.size() < population[second].verticesToRemove.size();
	});

	population[sortedFrontError.front()].distance = (float)INT_MAX;
	population[sortedFrontError.back()].distance = (float)INT_MAX;

	population[sortedFrontVertexCount.front()].distance = (float)INT_MAX;
	population[sortedFrontVertexCount.back()].distance = (float)INT_MAX;

	float maxVertexCountDifference = population[sortedFrontVertexCount.back()].verticesToRemove.size() - population[sortedFrontVertexCount.front()].verticesToRemove.size();
	float maxErrorDifference = population[sortedFrontVertexCount.back()].error - population[sortedFrontVertexCount.front()].error;

	if (maxVertexCountDifference == 0)
		maxVertexCountDifference = 1;

	if (glm::abs(maxErrorDifference) < 1e-5)
		maxErrorDifference = 1.f;

	for (int i = 1; i < paretoFront.size() - 1; ++i)
	{
		population[sortedFrontError[i]].distance += 
			(population[sortedFrontError[i+1]].error - population[sortedFrontError[i-1]].error)
			/ maxErrorDifference;

		population[sortedFrontVertexCount[i]].distance += 
			(population[sortedFrontVertexCount[i+1]].verticesToRemove.size() - population[sortedFrontVertexCount[i-1]].verticesToRemove.size())
			/ maxVertexCountDifference;
	}
}

void ModelOptimizer::plotParetoFronts(const TwoDimVector<uint32_t> &paretoFronts, const std::vector<Solution>& population, uint32_t originalVertexCount) const
{
	matplotlibcpp::figure();
	matplotlibcpp::title("Pareto fronts");
	matplotlibcpp::xlabel("Percentage of original vertices");
	matplotlibcpp::ylabel("Error");

	int counter = 1;
	for (const auto& front : paretoFronts)
	{
		std::vector<double> errors;
		std::vector<double> removedVerticesRatio;
		for (const uint32_t index : front)
		{
			errors.push_back(population[index].error);
			removedVerticesRatio.push_back((originalVertexCount - population[index].verticesToRemove.size()) * 100.f / originalVertexCount);
		}

		std::map<std::string, std::string> args({ { "label", std::to_string(counter++) + ". front" } });
		matplotlibcpp::scatter(removedVerticesRatio, errors, 10.0, args);
	}

	matplotlibcpp::legend();
	matplotlibcpp::show(true);
	std::cout << paretoFronts.size() << std::endl;
}

std::vector<ModelOptimizer::Solution> ModelOptimizer::nextGeneration(const std::vector<Solution> &population, const TwoDimVector<uint32_t> &paretoFronts)
{
	std::vector<Solution> newPopulation;
	for (const auto& front : paretoFronts)
	{	
		for (uint32_t index : front)
		{
			newPopulation.push_back(population[index]);
			if (newPopulation.size() == populationCount)
				return newPopulation;
		}
	}
}

void ModelOptimizer::printParetoFronts(const TwoDimVector<uint32_t>& paretoFronts, const std::vector<Solution>& population) const
{
	int counter = 1;
	for (const auto& front : paretoFronts)
	{
		std::cout << counter << ". front: \n";
		for (uint32_t index : front)
		{
			std::cout << '\t' << population[index].rank << ", " << population[index].distance << ", "
			<< population[index].error << ", " << population[index].verticesToRemove.size() << '\n';
		}
		std::cout << '\n';
		counter++;
	}
}
