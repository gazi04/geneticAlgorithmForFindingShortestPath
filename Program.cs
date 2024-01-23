//FIRST READ ALL THE COMMENTS WITH UPPER CASE FOR THE MAIN PROGRAM


using geneticAlgorithmFindingShortestPath;
using System.Runtime.CompilerServices;
Graph graph = new Graph();


////THIS IS THE FIRST EXAMPLE

//graph.AddNode("a");
//graph.AddNode("b");
//graph.AddNode("c");
//graph.AddNode("d");
//graph.AddNode("e");
//graph.AddNode("f");
//graph.AddNode("g");
//graph.AddNode("h");
//graph.AddNode("i");
//graph.AddNode("j");

//graph.AddEdge("a", "c", 3);
//graph.AddEdge("a", "f", 2);
//graph.AddEdge("b", "d", 1);
//graph.AddEdge("b", "f", 6);
//graph.AddEdge("b", "g", 2);
//graph.AddEdge("b", "j", 3);
//graph.AddEdge("c", "b", 4);
//graph.AddEdge("e", "d", 2);
//graph.AddEdge("f", "g", 5);
//graph.AddEdge("g", "j", 4);
//graph.AddEdge("h", "i", 2);
//graph.AddEdge("i", "j", 3);
//graph.AddEdge("d", "j", 2);
//graph.AddEdge("e", "i", 5);

////graph.PrintGraph();

//// It print shortest path by using dijkstras algorithm
//graph.PrintShortestPath("a", "h");
//Console.WriteLine();

//// It executes the genetic algorithm 10 times
//for (int i = 0; i < 10; i++)
//{
//    Console.WriteLine($"{i}#");
//    graph.RunGeneticAlgorithm(20, 10, "a", "h");
//}


// THE SECOND EXAMPLE WITH A DIFFERENT GRAPH
// Just a tip comment all the code above and decoment the code below from line 53 to line 87

graph.AddNode("a");
graph.AddNode("b");
graph.AddNode("c");
graph.AddNode("d");
graph.AddNode("e");
graph.AddNode("f");
graph.AddNode("g");

graph.AddEdge("a", "c", 3);
graph.AddEdge("a", "f", 2);
graph.AddEdge("b", "d", 1);
graph.AddEdge("b", "f", 6);
graph.AddEdge("b", "g", 2);
graph.AddEdge("c", "e", 1);
graph.AddEdge("c", "f", 2);
graph.AddEdge("c", "d", 4);
graph.AddEdge("e", "b", 2);
graph.AddEdge("f", "e", 3);
graph.AddEdge("f", "g", 5);

Console.WriteLine("Dijkstras algorithm: ");
graph.PrintShortestPath("a", "b");
Console.WriteLine();
Console.WriteLine("Generic Algorithm: ");
for (int i = 0; i < 10; i++)
{
    graph.RunGeneticAlgorithm(15, 3, "a", "b");
}


// IF YOU WANT AN AUTOMATED VERSION OF THE APPLICATION DECOMMENT ALL THE CODE BELOW AND COMMENT ALL THE CODE ABOVE
//Graph gr = new Graph();
//while (true)
//{
//    string input = Convert.ToString(Console.ReadLine());
//    input.ToLower();

//    if (input == "q") Environment.Exit(0);
//    else if (input == "help" || input == "h")
//    {
//        Console.WriteLine("COMANDS FOR THE APPLICATION");
//        Console.WriteLine("Q:\tQuit the program");
//        Console.WriteLine("H:\tGives the commands that can be used");
//        Console.WriteLine("R:\tGenerate random graph where you need to give the amount of nodes, edges and a range for the distance value");
//        Console.WriteLine("ADD:\tAdds a node into the graph");
//        Console.WriteLine("CON:\tConnects two node that are in the graph");
//        Console.WriteLine("DIJ:\tIt gives the shortest path between two nodes, through dijkstras algorithm");
//        Console.WriteLine("GA:\tIt gives us the most suitable solutions based on the genetic algorithm");
//        Console.WriteLine("PRINT:\tIt gives all the nodes with it neighbours and distance");
//    }
//    else if (input == "dij")
//    {
//        if (gr.GetNodes().Count > 0)
//        {
//            Console.WriteLine("First you need to give the starting node and then the destination.");
//            string start = Console.ReadLine();
//            string end = Console.ReadLine();
//            gr.PrintShortestPath(start, end);
//        }
//        else { Console.WriteLine("There are no nodes in the graph"); }
//    }
//    else if (input == "ga")
//    {
//        if (gr.GetNodes().Count > 0)
//        {
//            Console.WriteLine("You need to give me the size of the population, how many generations should be, the starting node and the destination node");
//            int populationSize = Convert.ToInt32(Console.ReadLine());
//            int genarations = Convert.ToInt32(Console.ReadLine());
//            string start = Console.ReadLine();
//            string end = Console.ReadLine();
//            gr.RunGeneticAlgorithm(populationSize, genarations, start, end);
//        }
//        else { Console.WriteLine("There are no nodes in the graph"); }
//    }
//    else if (input == "add")
//    {
//        gr.AddNode(Console.ReadLine());
//    }
//    else if (input == "con")
//    {
//        Console.WriteLine("Give the two nodes you want to connect with the weight");
//        string first = Console.ReadLine();
//        string second = Console.ReadLine();
//        int weight = int.Parse(Console.ReadLine());
//        gr.AddEdge(first, second, weight);
//    }
//    else if (input == "r")
//    {
//        Console.WriteLine("If you generate a random graph the graph that exsits is going to be delete. Do you want to continue, yes/no");
//        if (Console.ReadLine().ToLower() == "yes")
//        {
//            gr.Clear();
//            Console.WriteLine("Give the number of nodes, the number of edges, the minimal distance and the maximal distance");
//            int numNodes = Convert.ToInt32(Console.ReadLine());
//            int numEdges = Convert.ToInt32(Console.ReadLine());
//            int minDis = Convert.ToInt32(Console.ReadLine());
//            int maxDis = Convert.ToInt32(Console.ReadLine());
//            gr.GenerateRandomGraph(numNodes, numEdges, minDis, maxDis);
//        }
//    }
//    else if (input == "print") { gr.PrintGraph(); }
//}