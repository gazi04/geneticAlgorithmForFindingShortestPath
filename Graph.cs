using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.Intrinsics.X86;
using System.Security.Cryptography;
using System.Text;
using System.Threading.Tasks;
using static System.Runtime.InteropServices.JavaScript.JSType;

namespace geneticAlgorithmFindingShortestPath
{
    public class Graph
    {
        public List<Node> Nodes { get; set; }

        public Graph()
        {
            Nodes = new List<Node>();
        }

        public List<Node> GetNodes()
        {
            return this.Nodes;
        }

        public void AddNode(string value)
        {
            if (Nodes.Any(n => n.Value == value)) { return; }

            Nodes.Add(new Node(value));
        }

        public void RemoveNode(string value)
        {
            Node nodeToRemove = Nodes.FirstOrDefault(n => n.Value == value);

            if (nodeToRemove != null)
            {
                Nodes.Remove(nodeToRemove);

                // Disconnect the removed node from other nodes
                foreach (Node node in Nodes)
                {
                    node.Disconnect(nodeToRemove);
                }
            }
        }

        public void AddEdge(string fromValue, string toValue, int weight)
        {
            Node fromNode = Nodes.FirstOrDefault(n => n.Value == fromValue);
            Node toNode = Nodes.FirstOrDefault(n => n.Value == toValue);

            if (fromNode != null && toNode != null)
            {
                fromNode.ConnectWith(toNode, weight);
            }
        }

        public void RemoveEdge(string fromValue, string toValue)
        {
            Node fromNode = Nodes.FirstOrDefault(n => n.Value == fromValue);
            Node toNode = Nodes.FirstOrDefault(n => n.Value == toValue);

            if (fromNode != null && toNode != null)
            {
                fromNode.Disconnect(toNode);
            }
        }

        public void PrintGraph()
        {
            foreach (var node in this.Nodes)
            {
                node.PrintNeighbours();
            }
        }

        public void Clear()
        {
            // Deleting each connection(edge) between two nodes
            foreach (var node in Nodes)
            {
                node.Neighbours.Clear();
            }

            // Deleting the node it self that is stored in the graph
            Nodes.Clear();
        }

        public void Dijkstra(Node startNode, Node destinationNode)
        {
            Stopwatch stopwatch = Stopwatch.StartNew();

            // Initialize distances and predecessors
            foreach (Node node in Nodes)
            {
                node.Distance = int.MaxValue;
                node.Previous = null;
            }

            startNode.Distance = 0;

            // Priority queue to keep track of nodes with their distances
            List<Node> priorityQueue = new List<Node>(Nodes);

            while (priorityQueue.Count > 0)
            {
                // Extract node with the minimum distance
                Node nearestNeighbour = priorityQueue.OrderBy(n => n.Distance).First();
                priorityQueue.Remove(nearestNeighbour);

                // Stop the algorithm if the destination node is reached
                if (nearestNeighbour == destinationNode)
                {
                    break;
                }

                // Update distances and predecessors for neighboring nodes
                foreach (Tuple<Node, int> neighbor in nearestNeighbour.Neighbours)
                {
                    Node v = neighbor.Item1;
                    int altDistance = nearestNeighbour.Distance + neighbor.Item2;

                    if (altDistance < v.Distance)
                    {
                        v.Distance = altDistance;
                        v.Previous = nearestNeighbour;
                    }
                }
            }

            stopwatch.Stop();
            Console.WriteLine($"The execution time for dijkstra algorithm: {stopwatch.ElapsedMilliseconds} milliseconds");
            Console.WriteLine();
        }

        public void PrintShortestPath(string startNodeValue, string destinationNodeValue)
        {
            Node startNode = FindNode(startNodeValue);
            Node destinationNode = FindNode(destinationNodeValue);
            Dijkstra(startNode, destinationNode);

            Console.WriteLine($"Shortest path from Node {startNode.Value} to Node {destinationNode.Value}:");

            // Traverse the shortest path from destination to start
            Console.WriteLine("The path is traversed from destination to start");
            Node current = destinationNode;
            while (current != null)
            {
                Console.Write($"{current.Value} ");
                current = current.Previous;
            }

            Console.WriteLine($"\nTotal Distance: {destinationNode.Distance}");
        }

        public Node FindNode(string value)
        {
            return Nodes.FirstOrDefault(node => node.Value == value);
        }

        public void GenerateRandomGraph(int numNodes, int numEdges, int minDistance, int maxDistance)
        {
            if (numNodes <= 0 || numEdges < numNodes - 1 || numEdges > numNodes * (numNodes - 1) / 2)
            {
                throw new ArgumentException("Invalid input parameters for graph generation.");
            }

            Random rand = new Random();

            // Create nodes
            for (int i = 0; i < numNodes; i++)
            {
                AddNode($"Node-{i + 1}");
            }

            // Connect nodes randomly to form edges
            for (int edgeCount = 0; edgeCount < numEdges; edgeCount++)
            {
                Node node1 = Nodes[rand.Next(numNodes)];
                Node node2 = Nodes[rand.Next(numNodes)];

                // Ensure a node is not connected to itself and the edge does not already exist
                while (node1 == node2 || node1.Neighbours.Exists(n => n.Item1 == node2))
                {
                    node1 = Nodes[rand.Next(numNodes)];
                    node2 = Nodes[rand.Next(numNodes)];
                }

                // Assign a random weight within the specified range
                int weight = rand.Next(minDistance, maxDistance + 1);

                // Connect the nodes with the random weight
                node1.ConnectWith(node2, weight);
            }
        }

        // Generate an initial population of individuals for the genetic algorithm.
        public List<List<Node>> GenerateInitialPopulation(int populationSize, string start, string destination)
        {
            Node startNode = FindNode(start);
            Node destinationNode = FindNode(destination);
            List<List<Node>> population = new List<List<Node>>();

            for (int i = 0; i < populationSize; i++)
            {
                // Generate a random path as an individual
                List<Node> individual = GenerateRandomPath(startNode, destinationNode);

                population.Add(individual);
            }

            return population;
        }

        // Generate a random path in the graph from startNode to destinationNode.
        private List<Node> GenerateRandomPath(Node startNode, Node destinationNode)
        {
            Random rand = new Random();
            List<Node> paths = new List<Node>();
            Node current = startNode;

            paths.Add(current);

            // Randomly traverse the graph to generate a path
            while (current != destinationNode && rand.NextDouble() > 0.1) // Adjust the probability to control the length of paths
            {
                List<Node> neighbors = current.Neighbours.Select(neighbor => neighbor.Item1).ToList();

                if (neighbors.Count > 0)
                {
                    Node nextNode = neighbors[rand.Next(neighbors.Count)];
                    paths.Add(nextNode);
                    current = nextNode;
                }
                else
                {
                    break; // Break if there are no more neighbors
                }
            }

            // Ensure the path ends with the destinationNode
            if (paths.Last() != destinationNode)
            {
                paths.Add(destinationNode);
            }

            return paths;
        }

        // Evaluate the fitness of an individual (path) based on total distance.
        public int CalculateDistance(List<Node> path)
        {
            int totalDistance = 0;

            for (int i = 0; i < path.Count - 1; i++)
            {
                Node current = path[i];
                Node next = path[i + 1];

                // Check if there is a direct edge between current and next nodes
                bool connected = current.Neighbours.Any(n => n.Item1 == next);

                // If not connected, add zero distance; otherwise, add the edge weight
                //totalDistance += connected ? current.Neighbours.First(n => n.Item1 == next).Item2 : 0;

                if (connected)
                {
                    totalDistance += current.Neighbours.First(n => n.Item1 == next).Item2;
                }
                else return 0;
            }

            return totalDistance;
        }


        // Evaluate the fitness of each individual in the population.
        public Dictionary<List<Node>, int> FitnessEvaluation(List<List<Node>> population)
        {
            Dictionary<List<Node>, int> fitnessValues = new Dictionary<List<Node>, int>();

            foreach (var individual in population)
            {
                int fitness = CalculateDistance(individual);
                if(!fitnessValues.ContainsKey(individual))
                {
                    fitnessValues.Add(individual, fitness);
                }
            }

            return fitnessValues;
        }

        // Roulette wheel selection to choose individuals based on fitness.
        private List<List<Node>> RouletteWheelSelection(List<List<Node>> population, Dictionary<List<Node>, int> fitnessValues, int selectionCount)
        {
            List<List<Node>> selectedIndividuals = new List<List<Node>>();

            // Exclude individuals with zero fitness
            var nonZeroFitnessValues = fitnessValues.Where(kv => kv.Value > 0).ToDictionary(kv => kv.Key, kv => kv.Value);

            // Calculate total fitness of the non-zero fitness individuals.
            int totalFitness = nonZeroFitnessValues.Values.Sum();

            // Handle the case where totalFitness is zero to avoid division by zero.
            if (totalFitness == 0)
            {
                // Return an empty list or handle it based on your specific requirements.
                return selectedIndividuals;
            }

            // Calculate the probability of selection for each individual.
            // This ensures that lower fitness values result in higher selection probabilities.
            Dictionary<List<Node>, double> selectionProbabilities = nonZeroFitnessValues.ToDictionary(kv => kv.Key, kv => 1.0 - (double)kv.Value / totalFitness);

            // Perform selection.
            for (int i = 0; i < selectionCount; i++)
            {
                double randomValue = new Random().NextDouble();
                double cumulativeProbability = 0;

                foreach (var entry in selectionProbabilities)
                {
                    cumulativeProbability += entry.Value;

                    // Individuals are selected when randomValue is less than or equal to the cumulative probability, favoring individuals with lower fitness values
                    if (randomValue <= cumulativeProbability)
                    {
                        selectedIndividuals.Add(entry.Key);
                        break;
                    }
                }
            }

            return selectedIndividuals;
        }

        private List<List<Node>> RankBasedSelection(List<List<Node>> population, Dictionary<List<Node>, int> fitnessValues, int selectionCount)
        {
            List<List<Node>> selectedIndividuals = new List<List<Node>>();

            // Filter out individuals with zero fitness
            var nonZeroFitnessValues = fitnessValues.Where(kv => kv.Value > 0).ToDictionary(kv => kv.Key, kv => kv.Value);

            // Return an empty list if there are no individuals with non-zero fitness
            if (nonZeroFitnessValues.Count == 0)
            {
                return selectedIndividuals;
            }

            // Rank individuals based on their fitness values
            var rankedIndividuals = nonZeroFitnessValues.OrderBy(kv => kv.Value).ToList();

            // Calculate selection probabilities based on ranks
            Dictionary<List<Node>, double> selectionProbabilities = new Dictionary<List<Node>, double>();
            int rankSum = (rankedIndividuals.Count * (rankedIndividuals.Count + 1)) / 2;

            for (int i = 0; i < rankedIndividuals.Count; i++)
            {
                double probability = (double)(i + 1) / rankSum;
                selectionProbabilities.Add(rankedIndividuals[i].Key, probability);
            }

            // Perform selection
            for (int i = 0; i < selectionCount; i++)
            {
                double randomValue = new Random().NextDouble();
                double cumulativeProbability = 0;

                foreach (var entry in selectionProbabilities)
                {
                    cumulativeProbability += entry.Value;

                    // Individuals are selected when randomValue is less than or equal to the cumulative probability
                    if (randomValue <= cumulativeProbability)
                    {
                        selectedIndividuals.Add(entry.Key);
                        break;
                    }
                }
            }

            return selectedIndividuals;
        }

        public void RunGeneticAlgorithm(int populationSize, int generations, string startNode, string destinationNode)
        {
            Stopwatch stopwatch = Stopwatch.StartNew();
            // Generate an initial population of paths
            List<List<Node>> population = this.GenerateInitialPopulation(populationSize, startNode, destinationNode);

            for (int generation = 0; generation < generations; generation++)
            {
                //Console.WriteLine($"\tGen {generation}");

                // Evaluate the fitness of the current population
                Dictionary<List<Node>, int> fitnessValues = this.FitnessEvaluation(population);

                // Perform selection to choose individuals for the next generation
                List<List<Node>> selectedParents = RankBasedSelection(population, fitnessValues, populationSize);

                // Apply crossover to create offspring
                List<List<Node>> offspring = new List<List<Node>>();

                for (int i = 0; i < selectedParents.Count; i += 2)
                {
                    if (i + 1 < selectedParents.Count)
                    {
                        List<List<Node>> children = Crossover(selectedParents[i], selectedParents[i + 1]);
                        offspring.AddRange(children);
                    }
                    else
                    {
                        // If the number of selected parents is odd, one parent is left unaltered
                        offspring.Add(selectedParents[i]);
                    }
                }

                
                foreach (var individual in offspring)
                {
                    // For a population of 15 individual mutation rate should be 1%
                    Mutate(individual, 0.01);
                }


                // Replace the current population with the next generation
                population = offspring;

                // Filter out paths with fitness values of zero
                population = population.Where(path => this.CalculateDistance(path) > 0).ToList();

                // Sort the population based on fitness values
                population.Sort((path1, path2) => this.CalculateDistance(path1).CompareTo(this.CalculateDistance(path2)));

            }
            stopwatch.Stop();
            Console.WriteLine($"The execution time for the genetic algorithm: {stopwatch.ElapsedMilliseconds} milliseconds");

            // It print the individuals of the last generation
            foreach (var path in population)
            {
                Console.Write("Path: ");
                foreach (var node in path)
                {
                    Console.Write($"{node.Value} ");
                }
                Console.Write($"--->Cost: {this.CalculateDistance(path)}");
                Console.WriteLine();
                break;
            }
        }

        private List<List<Node>> Crossover(List<Node> parent1, List<Node> parent2)
        {
            // Select a random crossover point
            int crossoverPoint = new Random().Next(1, Math.Min(parent1.Count, parent2.Count));

            // Create two empty offspring
            List<Node> offspring1 = new List<Node>();
            List<Node> offspring2 = new List<Node>();

            // Add genetic material from parents up to the crossover point
            offspring1.AddRange(parent1.Take(crossoverPoint));
            offspring2.AddRange(parent2.Take(crossoverPoint));

            // Add the remaining genetic material by swapping parents
            offspring1.AddRange(parent2.Skip(crossoverPoint));
            offspring2.AddRange(parent1.Skip(crossoverPoint));

            return new List<List<Node>> { offspring1, offspring2 };
        }

        /// <summary>
        /// Considering factors for choosing mutation rate
        ///     Problem Complexity: More complex problems might benefit from higher mutation rates to explore a broader solution space.
        ///     Population Size: Smaller populations might benefit from higher mutation rates to inject more diversity.
        ///     Crossover Rate: If you have a high crossover rate, you might use a lower mutation rate, and vice versa.
        /// </summary>
        /// <param name="path">takes the path</param>
        /// <param name="mutationRate">the probability that the give path can be changed on a random point</param>
        private void Mutate(List<Node> path, double mutationRate)
        {
            Random rand = new Random();

            for (int i = 0; i < path.Count; i++)
            {
                // Apply mutation with a certain probability
                if (rand.NextDouble() < mutationRate)
                {
                    // Perform mutation - for example, randomly change the node at this position
                    int randomNodeIndex = rand.Next(path.Count);
                    path[i] = path[randomNodeIndex];
                }
            }
        }
    }
}
