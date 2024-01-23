using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace geneticAlgorithmFindingShortestPath
{
    public class Node
    {
        public string Value { get; set; }
        public List<Tuple<Node, int>> Neighbours { get; set; }
        public int Distance { get; set; }
        public Node Previous { get; set; }
        
        public Node(string vl)
        {
            this.Value = vl;
            this.Neighbours = new List<Tuple<Node, int>>();
            this.Distance = int.MaxValue;
            this.Previous = null;
        }

        public void ConnectWith(Node node, int weight)
        {
            if (Neighbours.Any(neighbour => neighbour.Item1 == node)) { return; }

            this.Neighbours.Add(new Tuple<Node, int>(node, weight)); 
            node.Neighbours.Add(new Tuple<Node, int>(this, weight));
        }

        public bool AdjacencyWith(Node node)
        {
            return Neighbours.Any(neighbour => neighbour.Item1 == node);
        }

        public void Disconnect(Node node)
        {
            Tuple<Node, int> connection = Neighbours.FirstOrDefault(n => n.Item1 == node);

            if (connection != null)
            {
                Neighbours.Remove(connection);
                // For bidirectional connection, also disconnect from the other node
                node.Neighbours.Remove(new Tuple<Node, int>(this, connection.Item2));
            }
        }

        public void PrintNeighbours()
        {
            Console.WriteLine($"Node {Value} has neighbors:");

            foreach (var neighbor in Neighbours)
            {
                Console.WriteLine($"  -> {neighbor.Item1.Value} with weight {neighbor.Item2}");
            }

            Console.WriteLine();
        }
    }
}
