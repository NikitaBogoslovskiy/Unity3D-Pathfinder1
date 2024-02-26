using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Grid : MonoBehaviour
{
    //  Модель для отрисовки узла сетки
    public GameObject nodeModel;

    //  Ландшафт (Terrain) на котором строится путь
    [SerializeField] private Terrain landscape = null;

    //  Шаг сетки (по x и z) для построения точек
    [SerializeField] private int gridDelta = 20;

    //  Номер кадра, на котором будет выполнено обновление путей
    private int updateAtFrame = 0;  

    //  Массив узлов - создаётся один раз, при первом вызове скрипта
    private PathNode[,] grid = null;

    private void CheckWalkableNodes()
    {
        foreach (PathNode node in grid)
        {
            //  Пока что считаем все вершины проходимыми, без учёта препятствий
            //node.walkable = true;
            node.walkable = !Physics.CheckSphere(node.body.transform.position, 1);
/*            if (node.walkable)
                node.Fade();
            else
            {
                node.Illuminate();
                Debug.Log("Not walkable!");
            }*/
        }
    }


    // Метод вызывается однократно перед отрисовкой первого кадра
    void Start()
    {
        //  Создаём сетку узлов для навигации - адаптивную, под размер ландшафта
        Vector3 terrainSize = landscape.terrainData.bounds.size;
        int sizeX = (int)(terrainSize.x / gridDelta);
        int sizeZ = (int)(terrainSize.z / gridDelta);
        //  Создаём и заполняем сетку вершин, приподнимая на 25 единиц над ландшафтом
        grid = new PathNode[sizeX,sizeZ];
        for (int x = 0; x < sizeX; ++x)
            for (int z = 0; z < sizeZ; ++z)
            {
                Vector3 position = new Vector3(x * gridDelta, 0, z * gridDelta);
                position.y = landscape.SampleHeight(position) + 25;
                grid[x, z] = new PathNode(nodeModel, false, position);
                grid[x, z].ParentNode = null;
                grid[x, z].Fade();
            }
    }
    /// <summary>
    /// Получение списка соседних узлов для вершины сетки
    /// </summary>
    /// <param name="current">индексы текущей вершины </param>
    /// <returns></returns>
    private List<Vector2Int> GetNeighbours(Vector2Int current)
    {
        List<Vector2Int> nodes = new List<Vector2Int>();
        for (int x = current.x - 1; x <= current.x + 1; ++x)
            for (int y = current.y - 1; y <= current.y + 1; ++y)
                if (x >= 0 && y >= 0 && x < grid.GetLength(0) && y < grid.GetLength(1) && (x != current.x || y != current.y))
                    nodes.Add(new Vector2Int(x, y));
                return nodes;
    }

    /// <summary>
    /// Вычисление "кратчайшего" между двумя вершинами сетки
    /// </summary>
    /// <param name="startNode">Координаты начального узла пути (индексы элемента в массиве grid)</param>
    /// <param name="finishNode">Координаты конечного узла пути (индексы элемента в массиве grid)</param>
    void calculatePath(Vector2Int startNode, Vector2Int finishNode)
    {
        //  Очищаем все узлы - сбрасываем отметку родителя, снимаем подсветку
        foreach (var node in grid)
        {
            node.Fade();
            node.ParentNode = null;
        }
        
        //  На данный момент вызов этого метода не нужен, там только устанавливается проходимость вершины. Можно добавить обработку препятствий
        CheckWalkableNodes();

        DijkstraSearchAlgorithm(startNode, finishNode);
        AStarSearchAlgorithm(startNode, finishNode);
        WaveSearchAlgorithm(startNode, finishNode);
    }

    private void DijkstraSearchAlgorithm(Vector2Int start, Vector2Int goal)
    {
        var frontier = new Utils.PriorityQueue<Vector2Int, float>();
        frontier.Enqueue(start, 0);
        var came_from = new Dictionary<Vector2Int, Vector2Int>();
        var cost_so_far = new Dictionary<Vector2Int, float>();
        came_from.Add(start, start);
        cost_so_far.Add(start, 0);

        var current = new Vector2Int(0, 0);
        while (frontier.Count > 0)
        {
            current = frontier.Dequeue();
            if (current == goal)
                break;
            foreach (var next in GetNeighbours(current))
            {
                if (!grid[next.x, next.y].walkable)
                    continue;
                float new_cost = cost_so_far[current] + PathNode.Dist(grid[next.x, next.y], grid[current.x, current.y]);
                if (!cost_so_far.ContainsKey(next) || new_cost < cost_so_far[next])
                {
                    cost_so_far[next] = new_cost;
                    float priority = new_cost;
                    frontier.Enqueue(next, priority);
                    came_from[next] = current;
                }
            }
        }

        while (current != start)
        {
            grid[current.x, current.y].Illuminate(Color.green);
            current = came_from[current];
        }
    }

    private void AStarSearchAlgorithm(Vector2Int start, Vector2Int goal)
    {
        var frontier = new Utils.PriorityQueue<Vector2Int, float>();
        frontier.Enqueue(start, 0);
        var came_from = new Dictionary<Vector2Int, Vector2Int>();
        var cost_so_far = new Dictionary<Vector2Int, float>();
        came_from.Add(start, start);
        cost_so_far.Add(start, 0);

        var current = new Vector2Int(0, 0);
        while (frontier.Count > 0)
        {
            current = frontier.Dequeue();
            if (current == goal) 
                break;
            foreach (var next in GetNeighbours(current))
            {
                if (!grid[next.x, next.y].walkable)
                    continue;
                float new_cost = cost_so_far[current] + PathNode.Dist(grid[next.x, next.y], grid[current.x, current.y]);
                if (!cost_so_far.ContainsKey(next) || new_cost < cost_so_far[next])
                {
                    cost_so_far[next] = new_cost;
                    float priority = new_cost + EuclidianDistance(goal, next, 5);
                    frontier.Enqueue(next, priority);
                    came_from[next] = current;
                }
            }
        }

        while (current != start)
        {
            grid[current.x, current.y].Illuminate(Color.red);
            current = came_from[current];
        }
    }

    private Vector3 GetCoordinates(Vector2Int gridNode)
    {
        var node = grid[gridNode.x, gridNode.y];
        return new Vector3(node.body.transform.position.x,
                           node.body.transform.position.y,
                           node.body.transform.position.z);
    }

    private Vector3 FindDeltas(Vector3 start, Vector3 end)
    {
        return new Vector3(Mathf.Abs(start.x - end.x),
                           Mathf.Abs(start.y - end.y),
                           Mathf.Abs(start.z - end.z));
    }

    private float ManhattanDistance(Vector2Int current, Vector2Int finish, float heightWeight = 1)
    {
        var deltas = FindDeltas(GetCoordinates(current), GetCoordinates(finish));
        var weightedHeight = heightWeight * deltas.y;
        return deltas.x + deltas.z + weightedHeight;
    }

    private float DiagonalDistance(Vector2Int current, Vector2Int finish, float heightWeight = 1)
    {
        var deltas = FindDeltas(GetCoordinates(current), GetCoordinates(finish));
        var weightedHeight = heightWeight * deltas.y;
        return (deltas.x + deltas.z + weightedHeight) + (Mathf.Sqrt(2) - 2) * Mathf.Min(deltas.x, deltas.z, weightedHeight);
    }

    private float EuclidianDistance(Vector2Int current, Vector2Int finish, float heightWeight = 1)
    {
        var deltas = FindDeltas(GetCoordinates(current), GetCoordinates(finish));
        var weightedHeight = heightWeight * deltas.y;
        return Mathf.Sqrt(deltas.x * deltas.x + deltas.z * deltas.z + weightedHeight * weightedHeight);
    }

    private void WaveSearchAlgorithm(Vector2Int startNode, Vector2Int finishNode)
    {
        //  Реализуется аналог волнового алгоритма, причём найденный путь не будет являться оптимальным 

        PathNode start = grid[startNode.x, startNode.y];

        //  Начальную вершину отдельно изменяем
        start.ParentNode = null;
        start.Distance = 0;

        //  Очередь вершин в обработке - в A* необходимо заменить на очередь с приоритетом
        Queue<Vector2Int> nodes = new Queue<Vector2Int>();
        //  Начальную вершину помещаем в очередь
        nodes.Enqueue(startNode);
        //  Пока не обработаны все вершины (очередь содержит узлы для обработки)
        while (nodes.Count != 0)
        {
            Vector2Int current = nodes.Dequeue();
            //  Если достали целевую - можно заканчивать (это верно и для A*)
            if (current == finishNode) break;
            //  Получаем список соседей
            var neighbours = GetNeighbours(current);
            foreach (var node in neighbours)
            {
                if (grid[node.x, node.y].walkable && grid[node.x, node.y].Distance > grid[current.x, current.y].Distance + PathNode.Dist(grid[node.x, node.y], grid[current.x, current.y]))
                {
                    grid[node.x, node.y].ParentNode = grid[current.x, current.y];
                    nodes.Enqueue(node);
                }
            }
        }

        //  Восстанавливаем путь от целевой к стартовой
        var pathElem = grid[finishNode.x, finishNode.y];
        while (pathElem != null)
        {
            pathElem.Illuminate(Color.yellow);
            pathElem = pathElem.ParentNode;
        }
    }

    // Метод вызывается каждый кадр
    void Update()
    {
        //  Чтобы не вызывать этот метод каждый кадр, устанавливаем интервал вызова в 1000 кадров
        if (Time.frameCount < updateAtFrame) return;
        updateAtFrame = Time.frameCount + 1000;

        calculatePath(new Vector2Int(0, 0), new Vector2Int(grid.GetLength(0)-1, grid.GetLength(1)-1));
    }
}
