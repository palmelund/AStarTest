using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Priority_Queue;

namespace AStarTestSuite
{
    public interface INode<T> where T : INode<T>
    {
        int X { get; set; }
        int Y { get; set; }
        bool CanEnter { get; set; }

        IEnumerable<T> Neighbors { get; set; }
    }
    
    public abstract class Pathfinder<T> : IEnumerable<T> where T : class, INode<T>, new()
    {
        public int Width { get; }
        public int Height { get; }

        private T[] _grid;
        private Random _rand;

        public Pathfinder(int width, int height, int blockChance, int seed = 1337)
        {
            Width = width;
            Height = width;

            _grid = new T[width * height];
            _rand = new Random(seed);

            for (int yIndex = 0; yIndex < height; yIndex++)
            {
                for (int xIndex = 0; xIndex < width; xIndex++)
                {
                    var canEnter = _rand.Next(0, 101) > blockChance;
                    _grid[yIndex * width + xIndex] = new T {X = xIndex, Y = yIndex, CanEnter = canEnter};
                }
            }

            _grid.First().CanEnter = true;
            _grid.Last().CanEnter = true;

            foreach (var node in _grid)
            {
                node.Neighbors = NonBlockingNeighbors(node);
            }
        }

        private IEnumerable<T> NonBlockingNeighbors(T node)
        {
            var lst = new List<T>();

            for (var y = -1; y <= 1; y++)
            {
                for (var x = -1; x <= 1; x++)
                {
                    if (x == 0 & y == 0)
                    {
                        continue;
                    }

                    lst.Add(GetNodeAt(node.X + x, node.Y + y));
                }
            }

            return lst.Where(n => n != null && n.CanEnter);
        }

        public T GetNodeAt(int x, int y)
        {
            if (x < 0 || y < 0 || x >= Width || y >= Height)
            {
                return null;
            }

            return _grid[y * Width + x];
        }

        protected static float HeuristicCostEstimate(T from, T to)
        {
            return Math.Abs(from.X - to.X) + Math.Abs(from.Y - to.Y);
        }

        public abstract IEnumerable<T> AStar(T from, T to);
        public IEnumerator<T> GetEnumerator()
        {
            return _grid.AsEnumerable().GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }
    
    

    // TODO: Fast priority queue might be slower than normal priority queue
    public class StandardPathFinder : Pathfinder<StandardPathFinder.Node>
    {
        public class Node : FastPriorityQueueNode, INode<Node>
        {
            public int X { get; set; }
            public int Y { get; set; }
            public bool CanEnter { get; set; }
            public IEnumerable<Node> Neighbors { get; set; }
        }

        public StandardPathFinder(int width, int height, int blockChance, int seed = 1337) : base(width, height,
            blockChance, seed)
        {
        }

        public override IEnumerable<Node> AStar(Node from, Node to)
        {
            var closedSet = new HashSet<Node>();
            var openSet = new FastPriorityQueue<Node>(Width * Height);

            openSet.Enqueue(@from, 0f);

            var cameFrom = new Dictionary<Node, Node>();

            var gScore = new Dictionary<Node, float> {{from, 0f}};

            while (openSet.Count > 0)
            {
                var currentNode = openSet.Dequeue();
                if (currentNode == to)
                {
                    return ReconstructPath(cameFrom, currentNode);
                }

                closedSet.Add(currentNode);

                foreach (var neighbor in currentNode.Neighbors)
                {
                    if (closedSet.Contains(neighbor))
                    {
                        continue;
                    }

                    var tentativeGScore = gScore[currentNode] + currentNode.Distance(neighbor);

                    if (openSet.Contains(neighbor) && tentativeGScore >= gScore[neighbor])
                    {
                        continue;
                    }

                    cameFrom[neighbor] = currentNode;
                    gScore[neighbor] = tentativeGScore;

                    var fs = tentativeGScore + HeuristicCostEstimate(neighbor, to);

                    if (openSet.Contains(neighbor))
                    {
                        openSet.UpdatePriority(neighbor, fs);
                    }
                    else
                    {
                        openSet.Enqueue(neighbor, fs);
                    }
                }
            }

            return new List<Node>();
        }

        private static IEnumerable<Node> ReconstructPath(IReadOnlyDictionary<Node, Node> cameFrom, Node currentNode)
        {
            var path = new Stack<Node>();
            path.Push(currentNode);

            while (cameFrom.ContainsKey(currentNode))
            {
                currentNode = cameFrom[currentNode];
                path.Push(currentNode);
            }

            return path;
        }
    }

    public class FastPathfinder : Pathfinder<FastPathfinder.Node>
    {
        public class Node : FastPriorityQueueNode, INode<Node>
        {
            public int X { get; set; }
            public int Y { get; set; }
            public bool CanEnter { get; set; }
            public IEnumerable<Node> Neighbors { get; set; }

            public bool InClosedSet { get; set; }
            public Node CameFrom { get; set; }
            public float GScore { get; set; } = float.PositiveInfinity;

            public void Reset()
            {
                InClosedSet = false;
                CameFrom = null;
                GScore = float.PositiveInfinity;
            }
        }

        public FastPathfinder(int width, int height, int blockChance, int seed = 1337) : base(width, height,
            blockChance, seed)
        {
        }

        public override IEnumerable<Node> AStar(Node from, Node to)
        {
            var openSet = new FastPriorityQueue<Node>(Width * Height);
            openSet.Enqueue(from, 0f);

            while (openSet.Count > 0)
            {
                var currentNode = openSet.Dequeue();
                if (currentNode == to)
                {
                    var path = ReconstructPath(currentNode);

                    foreach (var node in this)
                    {
                        node.Reset();
                    }

                    return path;
                }

                currentNode.InClosedSet = true;

                foreach (var neighbor in currentNode.Neighbors)
                {
                    if (neighbor.InClosedSet)
                    {
                        continue;
                    }

                    var tentativeGScore = currentNode.GScore + currentNode.Distance(neighbor);

                    if (openSet.Contains(neighbor) && tentativeGScore >= neighbor.GScore)
                    {
                        continue;
                    }

                    neighbor.CameFrom = currentNode;
                    neighbor.GScore = tentativeGScore;

                    var fs = tentativeGScore + HeuristicCostEstimate(neighbor, to);

                    if (openSet.Contains(neighbor))
                    {
                        openSet.UpdatePriority(neighbor, fs);
                    }
                    else
                    {
                        openSet.Enqueue(neighbor, fs);
                    }
                }
            }

            foreach (var node in this)
            {
                node.Reset();
            }
            
            return new List<Node>();
        }

        private static IEnumerable<Node> ReconstructPath(Node currentNode)
        {
            var path = new Stack<Node>();
            path.Push(currentNode);

            while (currentNode.CameFrom != null)
            {
                currentNode = currentNode.CameFrom;
                path.Push(currentNode);
            }

            return path;
        }
    }

    public class ParallelPathfinder : Pathfinder<ParallelPathfinder.Node>
    {
        public class Node : INode<Node>
        {
            public int X { get; set; }
            public int Y { get; set; }
            public bool CanEnter { get; set; }
            public IEnumerable<Node> Neighbors { get; set; }
        }

        public ParallelPathfinder(int width, int height, int blockChance, int seed = 1337) : base(width, height,
            blockChance, seed)
        {
        }

        public override IEnumerable<Node> AStar(Node from, Node to)
        {
            var closedSet = new ConcurrentBag<Node>();
            var openSet = new SimplePriorityQueue<Node>();

            openSet.Enqueue(from, 0f);

            var cameFrom = new ConcurrentDictionary<Node, Node>();

            var gScore = new ConcurrentDictionary<Node, float>();
            gScore.TryAdd(from, 0f);
            
            while (openSet.Count > 0)
            {
                var currentNode = openSet.Dequeue();
                if (currentNode == to)
                {
                    return ReconstructPath(cameFrom, currentNode);
                }

                closedSet.Add(currentNode);

                Parallel.ForEach(currentNode.Neighbors, neighbor =>
                {
                    if (closedSet.Contains(neighbor))
                    {
                        return;
                    }

                    var tentativeGScore = gScore[currentNode] + currentNode.Distance(neighbor);

                    if (openSet.Contains(neighbor) && tentativeGScore >= gScore[neighbor])
                    {
                        return;
                    }

                    cameFrom[neighbor] = currentNode;
                    gScore[neighbor] = tentativeGScore;

                    var fs = tentativeGScore + HeuristicCostEstimate(neighbor, to);

                    if (openSet.Contains(neighbor))
                    {
                        openSet.UpdatePriority(neighbor, fs);
                    }
                    else
                    {
                        openSet.Enqueue(neighbor, fs);
                    }
                });
            }

            return new List<Node>();
        }

        private static IEnumerable<Node> ReconstructPath(IReadOnlyDictionary<Node, Node> cameFrom, Node currentNode)
        {
            var path = new Stack<Node>();
            path.Push(currentNode);

            while (cameFrom.ContainsKey(currentNode))
            {
                currentNode = cameFrom[currentNode];
                path.Push(currentNode);
            }

            return path;
        }
    }

    public class Parallel2Pathfinder : Pathfinder<Parallel2Pathfinder.Node>
    {
        public class Node : INode<Node>
        {
            public int X { get; set; }
            public int Y { get; set; }
            public bool CanEnter { get; set; }
            public IEnumerable<Node> Neighbors { get; set; }

            public int ClosedSet = 0;
            public float GScore = float.PositiveInfinity;
            public Node CameFrom;

            public void Reset()
            {
                ClosedSet = 0;
                GScore = Single.PositiveInfinity;
                CameFrom = null;
            }
        }

        public Parallel2Pathfinder(int width, int height, int blockChance, int seed = 1337) : base(width, height, blockChance, seed)
        {
        }

        public override IEnumerable<Node> AStar(Node from, Node to)
        {
            var openSet = new SimplePriorityQueue<Node>();
            openSet.Enqueue(from, 0f);

            IEnumerable<Node> path = new List<Node>();
           
            while (openSet.Count > 0)
            {
                Parallel.For(0, Math.Min(4, openSet.Count), i =>
                {
                    var currentNode = openSet.Dequeue();
                    if (currentNode == to)
                    {
                        path = ReconstructPath(currentNode);
                    }

                    currentNode.ClosedSet = 1;

                    foreach (var neighbor in currentNode.Neighbors)
                    {
                        {
                            if (neighbor.ClosedSet != 0)
                            {
                                continue;
                            }

                            var tentativeGScore = currentNode.GScore + currentNode.Distance(neighbor);

                            if (openSet.Contains(neighbor) && tentativeGScore >= neighbor.GScore)
                            {
                                continue;
                            }

                            Interlocked.Exchange(ref neighbor.CameFrom, currentNode);
                            Interlocked.Exchange(ref neighbor.GScore, tentativeGScore);

                            var fs = tentativeGScore + HeuristicCostEstimate(neighbor, to);

                            if (openSet.Contains(neighbor))
                            {
                                openSet.UpdatePriority(neighbor, fs);
                            }
                            else
                            {
                                openSet.Enqueue(neighbor, fs);
                            }
                        }
                    }
                });
            }

            foreach (var node in this)
            {
                node.Reset();
            }
            
            return path;
        }

        private static IEnumerable<Node> ReconstructPath(Node currentNode)
        {
            var path = new Stack<Node>();
            path.Push(currentNode);

            while (currentNode.CameFrom != null)
            {
                currentNode = currentNode.CameFrom;
                path.Push(currentNode);
            }

            return path;
        }
    }

    public static class Program
    {
        private static void Main(string[] args)
        {
            TestSuite<StandardPathFinder, StandardPathFinder.Node>("Standard");
            TestSuite<FastPathfinder, FastPathfinder.Node>("Fast");
            TestSuite<ParallelPathfinder, ParallelPathfinder.Node>("Parallel");
            TestSuite<Parallel2Pathfinder, Parallel2Pathfinder.Node>("Parallel2");
        }
        
        private static void TestSuite<TPathfinder, TNode>(string testName)
            where TPathfinder : Pathfinder<TNode> where TNode : class, INode<TNode>, new()
        {
            const int gridSize = 100;
            const int blockChance = 30;
            const int seed = 1337;

            {

                var pathFinder =
                    Activator.CreateInstance(typeof(TPathfinder), gridSize, gridSize, blockChance, seed) as TPathfinder;

                GC.Collect();

                if (GC.TryStartNoGCRegion(1024 * 1024 * 128))
                {
                    var start = pathFinder.GetNodeAt(0, 0);
                    var finish = pathFinder.GetNodeAt(pathFinder.Width - 1, pathFinder.Height - 1);

                    var stopwatch = new Stopwatch();
                    stopwatch.Start();
                    var path = pathFinder.AStar(start, finish);
                    var res = stopwatch.ElapsedMilliseconds;

                    Console.WriteLine($"{testName} l: {path.Count()} t: {res} ms");

                    GC.EndNoGCRegion();
                }
                else
                {
                    throw new InsufficientMemoryException();
                }
            }
            
            GC.Collect();
        }
    }

//    public class Grid
//    {
//        private FastNode[] _fastNodes;
//        private SingleNode[] _singleNodes;
//        private ParallelNode[] _parallelNodes;
//        private AggregatorNode[] _aggregatorNodes;
//
//        public readonly int Width;
//        public readonly int Height;
//
//        public Grid(int width, int height, int blockChange, int seed)
//        {
//            Width = width;
//            Height = height;
//
//            _fastNodes = new FastNode[width * height];
//            _singleNodes = new SingleNode[width * height];
//            _parallelNodes = new ParallelNode[width * height];
//            _aggregatorNodes = new AggregatorNode[width * height];
//
//            var rand = new Random(seed);
//
//            for (var yIndex = 0; yIndex < height; yIndex++)
//            {
//                for (var xIndex = 0; xIndex < width; xIndex++)
//                {
//                    var canEnter = rand.Next(0, 101) > blockChange;
//                    _fastNodes[yIndex * width + xIndex] = new FastNode() {X = xIndex, Y = yIndex, CanEnter = canEnter};
//                    _singleNodes[yIndex * width + xIndex] = new SingleNode()
//                        {X = xIndex, Y = yIndex, CanEnter = canEnter};
//                    _parallelNodes[yIndex * width + xIndex] = new ParallelNode()
//                        {X = xIndex, Y = yIndex, CanEnter = canEnter};
//                    _aggregatorNodes[yIndex * width + xIndex] = new AggregatorNode()
//                        {X = xIndex, Y = yIndex, CanEnter = canEnter};
//                }
//            }
//
//            _fastNodes.First().CanEnter = true;
//            _fastNodes.Last().CanEnter = true;
//            _singleNodes.First().CanEnter = true;
//            _singleNodes.Last().CanEnter = true;
//            _parallelNodes.First().CanEnter = true;
//            _parallelNodes.Last().CanEnter = true;
//            _aggregatorNodes.First().CanEnter = true;
//            _aggregatorNodes.Last().CanEnter = true;
//
//            foreach (var node in _fastNodes)
//            {
//                node.Neighbors = NonBlockedNeighbors(_fastNodes, node);
//            }
//
//            foreach (var node in _singleNodes)
//            {
//                node.Neighbors = NonBlockedNeighbors(_singleNodes, node);
//            }
//
//            foreach (var node in _parallelNodes)
//            {
//                node.Neighbors = NonBlockedNeighbors(_parallelNodes, node);
//            }
//
//            foreach (var node in _aggregatorNodes)
//            {
//                node.Neighbors = NonBlockedNeighbors(_aggregatorNodes, node);
//            }
//        }
//
//        public IEnumerable<T> NonBlockedNeighbors<T>(T[] c, T node) where T : INode<T>
//        {
//            var lst = new List<T>();
//
//            for (var y = -1; y <= 1; y++)
//            {
//                for (var x = -1; x <= 1; x++)
//                {
//                    if (x == 0 & y == 0)
//                    {
//                        continue;
//                    }
//
//                    lst.Add(GetNodeAt(c, node.X + x, node.Y + y));
//                }
//            }
//
//            return lst.Where(n => n != null && n.CanEnter);
//        }
//
//        public T GetNodeAt<T>(T[] c, int x, int y) where T : INode<T>
//        {
//            if (x < 0 || y < 0 || x >= Width || y >= Height)
//            {
//                return default(T);
//            }
//
//            return c[y * Width + x];
//        }
//
//        public FastNode GetFastNodeAt(int x, int y)
//        {
//            return GetNodeAt(_fastNodes, x, y);
//        }
//
//        public SingleNode GetSingleNodeAt(int x, int y)
//        {
//            return GetNodeAt(_singleNodes, x, y);
//        }
//
//        public ParallelNode GetParallelNodeAt(int x, int y)
//        {
//            return GetNodeAt(_parallelNodes, x, y);
//        }
//
//        public AggregatorNode GetAggregatorNodeAt(int x, int y)
//        {
//            return GetNodeAt(_aggregatorNodes, x, y);
//        }
//
//        private static float HeuristicCostEstimate<T>(T from, T to) where T : INode<T>
//        {
//            return Math.Abs(from.X - to.X) + Math.Abs(from.Y - to.Y);
//        }
//
//        private static IEnumerable<SingleNode> ReconstructPath(SingleNode currentNode)
//        {
//            var path = new Stack<SingleNode>();
//            path.Push(currentNode);
//
//            while (currentNode.CameFrom != null)
//            {
//                currentNode = currentNode.CameFrom;
//                path.Push(currentNode);
//            }
//
//            return path;
//        }
//
//        private static IEnumerable<T> ReconstructPath<T>(IReadOnlyDictionary<T, T> cameFrom, T currentNode)
//            where T : INode<T>
//        {
//            var path = new Stack<T>();
//            path.Push(currentNode);
//
//            while (cameFrom.ContainsKey(currentNode))
//            {
//                currentNode = cameFrom[currentNode];
//                path.Push(currentNode);
//            }
//
//            return path;
//        }
//
//        public IEnumerable<FastNode> FastAStar(FastNode from, FastNode to)
//        {
//            var closedSet = new HashSet<FastNode>();
//            var openSet = new FastPriorityQueue<FastNode>(Width * Height);
//
//            openSet.Enqueue(from, 0f);
//
//            var cameFrom = new Dictionary<FastNode, FastNode>();
//
//            var gScore = new Dictionary<FastNode, float> {{from, 0f}};
//
//            while (openSet.Count > 0)
//            {
//                var currentNode = openSet.Dequeue();
//                if (currentNode == to)
//                {
//                    return ReconstructPath(cameFrom, currentNode);
//                }
//
//                closedSet.Add(currentNode);
//
//                foreach (var neighbor in currentNode.Neighbors)
//                {
//                    if (closedSet.Contains(neighbor))
//                    {
//                        continue;
//                    }
//
//                    var tentativeGScore = gScore[currentNode] + currentNode.Distance(neighbor);
//
//                    if (openSet.Contains(neighbor) && tentativeGScore >= gScore[neighbor])
//                    {
//                        continue;
//                    }
//
//                    cameFrom[neighbor] = currentNode;
//                    gScore[neighbor] = tentativeGScore;
//
//                    var fs = tentativeGScore + HeuristicCostEstimate(neighbor, to);
//
//                    if (openSet.Contains(neighbor))
//                    {
//                        openSet.UpdatePriority(neighbor, fs);
//                    }
//                    else
//                    {
//                        openSet.Enqueue(neighbor, fs);
//                    }
//                }
//            }
//
//            return new List<FastNode>();
//        }
//
//        public IEnumerable<SingleNode> SingleAStar(SingleNode from, SingleNode to)
//        {
//            //var closedSet = new HashSet<FastNode>();
//            var openSet = new FastPriorityQueue<SingleNode>(Width * Height);
//
//            openSet.Enqueue(from, 0f);
//
//            //var cameFrom = new Dictionary<FastNode, FastNode>();
//
//            //var gScore = new Dictionary<FastNode, float> {{from, 0f}};
//
//            while (openSet.Count > 0)
//            {
//                var currentNode = openSet.Dequeue();
//                if (currentNode == to)
//                {
//                    return ReconstructPath(currentNode);
//                }
//
//                currentNode.ClosedSet = true;
//
//                foreach (var neighbor in currentNode.Neighbors)
//                {
//                    if (neighbor.ClosedSet)
//                    {
//                        continue;
//                    }
//
//                    var tentativeGScore = currentNode.GScore + currentNode.Distance(neighbor);
//
//                    if (openSet.Contains(neighbor) && tentativeGScore >= neighbor.GScore)
//                    {
//                        continue;
//                    }
//
//                    neighbor.CameFrom = currentNode;
//                    neighbor.GScore = tentativeGScore;
//
//                    var fs = tentativeGScore + HeuristicCostEstimate(neighbor, to);
//
//                    if (openSet.Contains(neighbor))
//                    {
//                        openSet.UpdatePriority(neighbor, fs);
//                    }
//                    else
//                    {
//                        openSet.Enqueue(neighbor, fs);
//                    }
//                }
//            }
//
//            return new List<SingleNode>();
//        }
//
//        public IEnumerable<ParallelNode> ParallelAStar(ParallelNode from, ParallelNode to)
//        {
//            var closedSet = new ConcurrentBag<ParallelNode>();
//            var openSet = new SimplePriorityQueue<ParallelNode>();
//
//            openSet.Enqueue(from, 0f);
//
//            var cameFrom = new ConcurrentDictionary<ParallelNode, ParallelNode>();
//
//            var gScore = new ConcurrentDictionary<ParallelNode, float>();
//            gScore.TryAdd(from, 0f);
//
//            IEnumerable<ParallelNode> retVal = new List<ParallelNode>();
//
//            while (openSet.Count > 0)
//            {
//                Parallel.ForEach(openSet.DequeueMany(4), (node, state) =>
//                {
//                    if (node == to)
//                    {
//                        retVal = ReconstructPath(cameFrom, node);
//                        state.Stop();
//                    }
//
//                    closedSet.Add(node);
//
//                    foreach (var neighbor in node.Neighbors)
//                    {
//                        if (closedSet.Contains(neighbor))
//                        {
//                            continue;
//                        }
//
//                        var tentativeGScore = gScore[node] + node.Distance(neighbor);
//
//                        if (openSet.Contains(neighbor) && tentativeGScore >= gScore[neighbor])
//                        {
//                            continue;
//                        }
//
//                        cameFrom[neighbor] = node;
//                        gScore[neighbor] = tentativeGScore;
//
//                        var fs = tentativeGScore + HeuristicCostEstimate(neighbor, to);
//
//                        if (openSet.Contains(neighbor))
//                        {
//                            openSet.UpdatePriority(neighbor, fs);
//                        }
//                        else
//                        {
//                            openSet.Enqueue(neighbor, fs);
//                        }
//                    }
//                });
//            }
//
//            return retVal;
//        }
//
//        public IEnumerable<ParallelNode> Parallel2AStar(ParallelNode from, ParallelNode to)
//        {
//            var closedSet = new ConcurrentBag<ParallelNode>();
//            var openSet = new SimplePriorityQueue<ParallelNode>();
//
//            openSet.Enqueue(from, 0f);
//
//            var cameFrom = new ConcurrentDictionary<ParallelNode, ParallelNode>();
//
//            var gScore = new ConcurrentDictionary<ParallelNode, float>();
//            gScore.TryAdd(from, 0f);
//
//            IEnumerable<ParallelNode> retVal = new List<ParallelNode>();
//
//            while (openSet.Count > 0)
//            {
//                var node = openSet.Dequeue();
//
//                if (node == to)
//                {
//                    retVal = ReconstructPath(cameFrom, node);
//                }
//
//                closedSet.Add(node);
//
//
//
//                Parallel.ForEach(node.Neighbors, neighbor =>
//                {
//                    if (closedSet.Contains(neighbor))
//                    {
//                        return;
//                    }
//
//                    var tentativeGScore = gScore[node] + node.Distance(neighbor);
//
//                    if (openSet.Contains(neighbor) && tentativeGScore >= gScore[neighbor])
//                    {
//                        return;
//                    }
//
//                    cameFrom[neighbor] = node;
//                    gScore[neighbor] = tentativeGScore;
//
//                    var fs = tentativeGScore + HeuristicCostEstimate(neighbor, to);
//
//                    if (openSet.Contains(neighbor))
//                    {
//                        openSet.UpdatePriority(neighbor, fs);
//                    }
//                    else
//                    {
//                        openSet.Enqueue(neighbor, fs);
//                    }
//                });
//            }
//
//
//            return retVal;
//        }
//    }

//    public class SingleNode : FastPriorityQueueNode, INode<SingleNode>
//    {
//        public int X { get; set; }
//        public int Y { get; set; }
//        public bool CanEnter { get; set; }
//        public IEnumerable<SingleNode> Neighbors { get; set; }
//
//        public bool ClosedSet { get; set; } = false;
//        public float GScore { get; set; }
//        public SingleNode CameFrom { get; set; }
//    }
//
//    public class FastNode : FastPriorityQueueNode, INode<FastNode>
//    {
//        public int X { get; set; }
//        public int Y { get; set; }
//        public bool CanEnter { get; set; }
//        public IEnumerable<FastNode> Neighbors { get; set; }
//    }
//
//    public class ParallelNode : INode<ParallelNode>
//    {
//        public int X { get; set; }
//        public int Y { get; set; }
//        public bool CanEnter { get; set; }
//        public IEnumerable<ParallelNode> Neighbors { get; set; }
//    }
//
//    public class AggregatorNode : INode<AggregatorNode>
//    {
//        public int X { get; set; }
//        public int Y { get; set; }
//        public bool CanEnter { get; set; }
//        public IEnumerable<AggregatorNode> Neighbors { get; set; }
//    }
//
//    public interface INode<T> where T : INode<T>
//    {
//        int X { get; }
//        int Y { get; }
//
//        bool CanEnter { get; }
//
//        IEnumerable<T> Neighbors { get; set; }
//    }

    public static class Extensions
    {
        public static float Distance<T>(this T node, T other) where T : INode<T>
        {
            return MathF.Sqrt(MathF.Pow(other.X - node.X, 2) + MathF.Pow(other.Y - node.Y, 2));
        }

//
//        public static IEnumerable<T> DequeueMany<T>(this ConcurrentQueue<T> c, int x)
//        {
//            var res = new List<T>();
//
//            while (!c.IsEmpty && res.Count < x)
//            {
//                var b = c.TryDequeue(out var t);
//                if (b)
//                {
//                    res.Add(t);
//                }
//            }
//
//            return res;
//        }
//
//        public static IEnumerable<T> DequeueMany<T>(this SimplePriorityQueue<T> q, int x)
//        {
//            var res = new List<T>();
//
//            while (q.Count > 0 && res.Count < x)
//            {
//                res.Add(q.Dequeue());
//            }
//
//            return res;
//        }
    }
}