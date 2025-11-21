using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using TMPro;
using Unity.Collections;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using System.Runtime.InteropServices;

namespace BSPhysics
{
    //为了规避执行时的计算压力，能在生成时预计算的都预先计算好
    public class TetAsset : ScriptableObject
    {
        public static float GetEdgeLength(int2 edge, float4[] vertexDatas)
        {
            float3 p0 = vertexDatas[edge.x].xyz;
            float3 p1 = vertexDatas[edge.y].xyz;
            return math.length(p0 - p1);
        }
        // [HideInInspector]
        public int3 CellOrigin;
        // [HideInInspector]
        public float3 CellSize;
        // [HideInInspector]
        public int3 CellCount;
        public float4[] vertices;
        public Tet[] tetras;
        //为了逐边计算约束关系还需要维护每条边的顶点编号，为了去重强制第一位是序号小的
        public Edge[] edges;
        //由于逐Tet计算在GPU中会存在竞争(一个顶点可能被多个tet共有)，不同tet的计算结果都需要影响同一个顶点
        //所幸顶点与tet的绑定关系固定，这里将映射关系展平成两张表；后面边也同理
        //第一张表记录所有顶点对应的tet编号，第二张表记录每个顶点对应的偏移，每个顶点对应tet的数量可以通过后一个减前一个
        public uint[] vertsToTets;//总长度是tet数量*4
        public int[] vertsToTetsOffset;//总长度是vert数量+1
        public uint[] vertsToEdges;//总长度是edge数量*2
        public int[] vertsToEdgesOffset;//总长度是vert数量+1

        public uint[] cellsToTets;//空间网格映射到四面体，长度不定
        public int[] cellsToTetsOffset;//空间网格偏移，长度为网格总数+1

        public int meshHash;
        public VtxResult[] vtxResults;

        //常态由每个TetAsset管理的Buffers，共九个
        private ComputeBuffer tetVtxOrginBuffer;
        private ComputeBuffer tetsBuffer;
        private ComputeBuffer edgesBuffer;
        private ComputeBuffer vertsToEdgesBuffer;
        private ComputeBuffer vertsToEdgesOffsetBuffer;
        private ComputeBuffer vertsToTetsBuffer;
        private ComputeBuffer vertsToTetsOffsetBuffer;
        private ComputeBuffer cellsToTetsBuffer;
        private ComputeBuffer cellsToTetsOffsetBuffer;
        private ComputeBuffer meshVtxArgsBuffer;
        private List<ComputeBuffer> localBufferList = new List<ComputeBuffer>();
        int3 WorldToGrid(float3 worldPos, float3 gridCellSize)
        {
            return new int3(
                Mathf.FloorToInt(worldPos.x / gridCellSize.x),
                Mathf.FloorToInt(worldPos.y / gridCellSize.y),
                Mathf.FloorToInt(worldPos.z / gridCellSize.z)
            );
        }

        public void DrawGizmo(Transform transform)
        {
            foreach (var edge in edges)
            {
                Gizmos.color = Color.yellow;
                int2 index = edge.index;
                DrawLine(vertices[index.x], vertices[index.y], transform);
            }
        }
        public void SetData(float4[] verticesData, int4[] tetsData)
        {
            vertices = verticesData;
            tetras = tetsData.Select(x => new Tet { index = x, volume = TetraBuilder.GetTetVolume(x, vertices) }).ToArray();
            int tetCount = tetras.Length;
            int vertexCount = vertices.Length;
            // 1. 预分配哈希表和字典容量
            var edgesHash = new HashSet<Edge>(capacity: 6 * tetCount);
            var edgesList = new List<Edge>(capacity: 6 * tetCount);
            var vertexToTetraMap = new Dictionary<int, List<int>>(vertexCount);
            var vertexToEdgeMap = new Dictionary<int, List<int>>(vertexCount * 2);
            var cellsToTetsMap = new Dictionary<int3, List<int>>();
            int3 cellIndexMin = new int3(1, 1, 1) * int.MaxValue;
            int3 cellIndexMax = new int3(-1, -1, -1) * int.MaxValue;
            // 2. 生成边、顶点到四面体/边映射,同时生成四面体/边到顶点位置的映射
            for (int i = 0; i < tetCount; i++)
            {
                int4 tetra = tetras[i].index;
                for (int j = 0; j < 4; j++)
                {
                    int vertex = tetra[j];

                    // 更新顶点到四面体映射
                    List<int> tetList;
                    if (!vertexToTetraMap.TryGetValue(vertex, out tetList))
                    {
                        tetList = new List<int>(4); // 每个顶点关联的四面体数通常较小
                        vertexToTetraMap.Add(vertex, tetList);
                    }
                    tetList.Add(i);
                    // 生成边并更新顶点到边映射
                    for (int k = 0; k < j; k++)
                    {
                        int v1 = tetra[j];
                        int v2 = tetra[k];
                        Edge edge = new Edge(v1, v2);
                        if (edgesHash.Add(edge))
                        {
                            edgesList.Add(edge);
                            int edgeIndex = edgesList.Count - 1;
                            // 更新顶点到边映射（v1）
                            List<int> edgeListV1;
                            if (!vertexToEdgeMap.TryGetValue(v1, out edgeListV1))
                            {
                                edgeListV1 = new List<int>(6); // 每个顶点关联的边数通常为6
                                vertexToEdgeMap.Add(v1, edgeListV1);
                            }
                            edgeListV1.Add(edgeIndex);
                            // 更新顶点到边映射（v2）
                            List<int> edgeListV2;
                            if (!vertexToEdgeMap.TryGetValue(v2, out edgeListV2))
                            {
                                edgeListV2 = new List<int>(6);
                                vertexToEdgeMap.Add(v2, edgeListV2);
                            }
                            edgeListV2.Add(edgeIndex);
                        }
                    }
                }
                //计算空间网格，插入空间网格列表
                float3 tetMin = math.min(math.min(vertices[tetra.x].xyz, vertices[tetra.y].xyz), math.min(vertices[tetra.z].xyz, vertices[tetra.w].xyz));
                float3 tetMax = math.max(math.max(vertices[tetra.x].xyz, vertices[tetra.y].xyz), math.max(vertices[tetra.z].xyz, vertices[tetra.w].xyz));
                //添加安全边界,这里使用半个cell的值
                float3 border = CellSize * 0.5f;
                tetMin -= border;
                tetMax += border;
                int3 minCell = WorldToGrid(tetMin, CellSize);
                int3 maxCell = WorldToGrid(tetMax, CellSize);
                cellIndexMin = math.min(minCell, cellIndexMin);
                cellIndexMax = math.max(maxCell, cellIndexMax);
                // 将四面体索引注册到所有覆盖的网格单元
                for (int x = minCell.x; x <= maxCell.x; x++)
                    for (int y = minCell.y; y <= maxCell.y; y++)
                        for (int z = minCell.z; z <= maxCell.z; z++)
                        {
                            int3 cell = new int3(x, y, z);
                            if (!cellsToTetsMap.ContainsKey(cell)) cellsToTetsMap[cell] = new List<int>();
                            cellsToTetsMap[cell].Add(i);
                        }
            }
            int edgeCount = edgesList.Count;
            edges = edgesList.Select(x => new Edge { index = x.index, length = GetEdgeLength(x.index, verticesData) }).ToArray();
            // 3. 预计算列表容量（替代 Sum）
            int totalVertsToTets = 0;
            foreach (var list in vertexToTetraMap.Values)
            {
                totalVertsToTets += list.Count;
            }
            List<uint> vertsToTetsList = new List<uint>(totalVertsToTets);
            int totalVertsToEdges = 0;
            foreach (var list in vertexToEdgeMap.Values)
            {
                totalVertsToEdges += list.Count;
            }
            List<uint> vertsToEdgesList = new List<uint>(totalVertsToEdges);
            int totalCellsToTets = 0;
            foreach (var list in cellsToTetsMap.Values)
            {
                totalCellsToTets += list.Count;
            }
            List<uint> cellsTotetsList = new List<uint>(totalCellsToTets);
            // 4. 填充扁平化结构
            int[] vertexToTetOffset = new int[vertexCount + 1];
            int[] vertexToEdgeOffset = new int[vertexCount + 1];
            var cellIndexSize = cellIndexMax - cellIndexMin + new int3(1, 1, 1);
            var cellCount = cellIndexSize.x * cellIndexSize.y * cellIndexSize.z;
            Debug.Log($"当前尺寸共划分空间网格{cellCount}个");
            int[] cellToTetOffset = new int[cellCount + 1]; ;//由于tet构建时对AABB做了微小偏移，所以无法预计算
            for (int i = 0; i < vertexCount; i++)
            {
                vertexToTetOffset[i] = vertsToTetsList.Count;
                if (vertexToTetraMap.TryGetValue(i, out var tetList))
                {
                    foreach (uint tet in tetList)
                    {
                        //左移2位存入偏移索引
                        vertsToTetsList.Add(tet << 2 | OffsetOf(tetras[tet].index, i));
                    }
                }
                vertexToEdgeOffset[i] = vertsToEdgesList.Count;
                if (vertexToEdgeMap.TryGetValue(i, out var edgeList))
                {
                    foreach (uint edge in edgeList)
                    {
                        //左移1位存入偏移索引
                        vertsToEdgesList.Add(edge << 1 | OffsetOf(edges[edge].index, i));
                    }
                }
            }
            int count = 0;
            for (int i = 0; i < cellIndexSize.x; i++)
                for (int j = 0; j < cellIndexSize.y; j++)
                    for (int k = 0; k < cellIndexSize.z; k++)
                    {
                        var index = i * cellIndexSize.y * cellIndexSize.z + j * cellIndexSize.z + k;
                        int3 cell = cellIndexMin + new int3(i, j, k);
                        cellToTetOffset[index] = cellsTotetsList.Count;
                        if (cellsToTetsMap.TryGetValue(cell, out var tetList))
                        {
                            foreach (uint tet in tetList)
                            {
                                cellsTotetsList.Add(tet);
                                count = math.max(count, tetList.Count);
                            }
                        }
                    }
            Debug.Log($"单个网格的最大四面体数为{count}");
            vertexToTetOffset[vertexCount] = vertsToTetsList.Count;
            vertexToEdgeOffset[vertexCount] = vertsToEdgesList.Count;
            cellToTetOffset[cellCount] = cellsTotetsList.Count;
            // 输出结果
            CellOrigin = cellIndexMin;
            CellCount = cellIndexSize;
            vertsToTets = vertsToTetsList.ToArray();
            vertsToTetsOffset = vertexToTetOffset;
            vertsToEdges = vertsToEdgesList.ToArray();
            vertsToEdgesOffset = vertexToEdgeOffset;
            cellsToTets = cellsTotetsList.ToArray();
            cellsToTetsOffset = cellToTetOffset;
        }
        void DrawLine(float4 a, float4 b, Transform transform)
        {
            Gizmos.DrawLine(transform.TransformPoint(a.xyz), transform.TransformPoint(b.xyz));
        }
        uint OffsetOf(int4 tetIndex, int vertIndex)
        {
            for (int i = 0; i < 4; i++)
            {
                if (tetIndex[i] == vertIndex)
                    return (uint)i;
            }
            Debug.LogError("构造错误");
            return 0;
        }
        uint OffsetOf(int2 edgeIndex, int vertIindex)
        {
            for (int i = 0; i < 2; i++)
            {
                if (edgeIndex[i] == vertIindex)
                    return (uint)i;
            }
            Debug.LogError("构造错误");
            return 0;
        }
        public float4[] GetTetMeshVtxData()
        {
            return vertices;
        }
        public void CalculateMeshArgsInCS(ComputeShader CS, Mesh mesh, int hash)
        {
            int meshVtxCount = mesh.vertices.Length;
            int tetsCount = tetras.Length;
            int cellsToTetsCount = cellsToTets.Length;
            int cellsCount = cellsToTetsOffset.Length;
            vtxResults = new VtxResult[meshVtxCount];

            int kernelGetArgsID = CS.FindKernel(ShaderParams.KernelGetMeshVtxArgs);
            CS.GetKernelThreadGroupSizes(kernelGetArgsID, out var CountThreadGroups, out var _, out var _);

            mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
            var meshStride = mesh.GetVertexBufferStride(0);
            var meshVertexBuffer = mesh.GetVertexBuffer(0);
            var tetVtxPosBuffer = new ComputeBuffer(meshVtxCount, Marshal.SizeOf<float4>());
            var meshVtxArgsBuffer = new ComputeBuffer(meshVtxCount, Marshal.SizeOf<VtxResult>());
            tetsBuffer = new ComputeBuffer(tetsCount, Marshal.SizeOf<Tet>());
            cellsToTetsBuffer = new ComputeBuffer(cellsToTetsCount, Marshal.SizeOf<int>());
            cellsToTetsOffsetBuffer = new ComputeBuffer(cellsCount + 1, Marshal.SizeOf<int>());

            localBufferList.Add(meshVtxArgsBuffer);
            localBufferList.Add(tetVtxPosBuffer);
            localBufferList.Add(tetsBuffer);
            localBufferList.Add(cellsToTetsBuffer);
            localBufferList.Add(cellsToTetsOffsetBuffer);
            localBufferList.Add(cellsToTetsOffsetBuffer);

            //SetData
            tetVtxPosBuffer.SetData(vertices);
            tetsBuffer.SetData(tetras);
            cellsToTetsBuffer.SetData(cellsToTets);
            cellsToTetsOffsetBuffer.SetData(cellsToTetsOffset);
            CS.SetBuffer(kernelGetArgsID,ShaderParams.MeshVtxBufID,meshVertexBuffer);
            CS.SetBuffer(kernelGetArgsID, ShaderParams.TetsBufID, tetsBuffer);
            CS.SetBuffer(kernelGetArgsID,ShaderParams.TetVtxPosBufID,tetVtxPosBuffer);
            CS.SetBuffer(kernelGetArgsID, ShaderParams.CellsToTetsBufID, cellsToTetsBuffer);
            CS.SetBuffer(kernelGetArgsID, ShaderParams.CellsToTetsOffsetBufID, cellsToTetsOffsetBuffer);
            CS.SetBuffer(kernelGetArgsID,ShaderParams.MeshVtxArgsBufID,meshVtxArgsBuffer);

            CS.SetInt(ShaderParams.MeshVtxStrideID, meshStride);
            int[] cellOriginArray = new int[4] { CellOrigin.x, CellOrigin.y, CellOrigin.z, 0 };
            CS.SetInts(ShaderParams.CellOriginArrayID, cellOriginArray);
            CS.SetVector(ShaderParams.CellSizeVectorID, new Vector4(CellSize.x, CellSize.y, CellSize.z, 0));
            int[] cellCountArray = new int[4] { CellCount.x, CellCount.y, CellCount.z, 0 };
            CS.SetInts(ShaderParams.CellCountArrayID, cellCountArray);

            CS.Dispatch(kernelGetArgsID, (int)math.ceil(meshVtxCount / (float)CountThreadGroups), 1, 1);

            meshHash = hash;
            meshVtxArgsBuffer.GetData(vtxResults);

            
            meshVertexBuffer.Dispose();
            meshVertexBuffer.Release();
            ReleaseAssetBuffer();
        }
        public void CreateSimulateBuffer(ComputeShader CS)
        {
            int kernelSetBufferDataID = CS.FindKernel(ShaderParams.KernelSetBufferData);
            int kernelSolveEdgesID = CS.FindKernel(ShaderParams.KernelSolveEdges);
            int kernelSolveVolumesID = CS.FindKernel(ShaderParams.KernelSolveVolumes);
            int kernelMergeResultsID = CS.FindKernel(ShaderParams.KernelMergeResults);
            int KernelApplyMeshVertex = CS.FindKernel(ShaderParams.KernelApplyMeshVertex);
            int tetMeshVtxCount = vertices.Length;
            int tetsCount = tetras.Length;
            int edgesCount = edges.Length;
            int meshVtxCount = vtxResults.Length;

            //Initial
            tetVtxOrginBuffer = new ComputeBuffer(tetMeshVtxCount,Marshal.SizeOf<float4>());
            tetsBuffer = new ComputeBuffer(tetsCount, Marshal.SizeOf<Tet>());
            edgesBuffer = new ComputeBuffer(edgesCount, Marshal.SizeOf<Edge>());
            vertsToEdgesBuffer = new ComputeBuffer(edgesCount * 2, Marshal.SizeOf<int>());
            vertsToEdgesOffsetBuffer = new ComputeBuffer(tetMeshVtxCount + 1, Marshal.SizeOf<int>());
            vertsToTetsBuffer = new ComputeBuffer(tetsCount * 4, Marshal.SizeOf<int>());
            vertsToTetsOffsetBuffer = new ComputeBuffer(tetMeshVtxCount + 1, Marshal.SizeOf<int>());
            meshVtxArgsBuffer = new ComputeBuffer(meshVtxCount, Marshal.SizeOf<VtxResult>());

            localBufferList.Add(tetVtxOrginBuffer);
            localBufferList.Add(meshVtxArgsBuffer);
            localBufferList.Add(tetsBuffer);
            localBufferList.Add(edgesBuffer);
            localBufferList.Add(vertsToEdgesBuffer);
            localBufferList.Add(vertsToEdgesOffsetBuffer);
            localBufferList.Add(vertsToTetsBuffer);
            localBufferList.Add(vertsToTetsOffsetBuffer);
            //SetData
            tetVtxOrginBuffer.SetData(vertices);
            tetsBuffer.SetData(tetras);
            edgesBuffer.SetData(edges);
            vertsToEdgesBuffer.SetData(vertsToEdges);
            vertsToEdgesOffsetBuffer.SetData(vertsToEdgesOffset);
            vertsToTetsBuffer.SetData(vertsToTets);
            vertsToTetsOffsetBuffer.SetData(vertsToTetsOffset);
            meshVtxArgsBuffer.SetData(vtxResults);

            CS.SetBuffer(kernelSetBufferDataID,ShaderParams.TetVTXOrginBufID,tetVtxOrginBuffer);

            CS.SetBuffer(kernelSolveEdgesID, ShaderParams.EdgesBufID, edgesBuffer);
            CS.SetBuffer(kernelSolveEdgesID, ShaderParams.VertsToEdgesBufID, vertsToEdgesBuffer);
            CS.SetBuffer(kernelSolveEdgesID, ShaderParams.VertsToEdgesOffsetBufID, vertsToEdgesOffsetBuffer);

            CS.SetBuffer(kernelSolveVolumesID, ShaderParams.TetsBufID, tetsBuffer);
            CS.SetBuffer(kernelSolveVolumesID, ShaderParams.VertsToTetsBufID, vertsToTetsBuffer);
            CS.SetBuffer(kernelSolveVolumesID, ShaderParams.VertsToTetsOffsetBufID, vertsToTetsOffsetBuffer);

            CS.SetBuffer(kernelMergeResultsID, ShaderParams.VertsToEdgesBufID, vertsToEdgesBuffer);
            CS.SetBuffer(kernelMergeResultsID, ShaderParams.VertsToEdgesOffsetBufID, vertsToEdgesOffsetBuffer);
            CS.SetBuffer(kernelMergeResultsID, ShaderParams.VertsToTetsBufID, vertsToTetsBuffer);
            CS.SetBuffer(kernelMergeResultsID, ShaderParams.VertsToTetsOffsetBufID, vertsToTetsOffsetBuffer);

            CS.SetBuffer(KernelApplyMeshVertex, ShaderParams.MeshVtxArgsBufID, meshVtxArgsBuffer);
            CS.SetBuffer(KernelApplyMeshVertex, ShaderParams.TetsBufID, tetsBuffer);
        }
        public void ReleaseAssetBuffer()
        {
            foreach (ComputeBuffer buffer in localBufferList)
            {
                buffer?.Dispose();
                buffer?.Release();
            }
        }
        public ComputeBuffer GetEdgesBuffer()
        {
            return edgesBuffer;
        }
    }
}
