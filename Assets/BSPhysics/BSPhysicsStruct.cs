using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.Mathematics;

namespace BSPhysics
{
    //构造BVH用
    [Serializable]
    public struct Triangle
    {
        public float3 Pos0;
        public float3 Pos1;
        public float3 Pos2;
        public float3 normal;
    }
    public struct AABB
    {
        public float3 Min;
        public float3 Max;
        public float3 Size//对角向量
        { get { return Max - Min; } }
        public float3 Center
        { get { return (Max + Min) * 0.5f; } }
        public AABB(float3 min, float3 max)
        {
            Min = min;
            Max = max;
        }
    }
    public struct TriangleData
    {
        public AABB TriangleAABB;
        public int TriangleIndex;
    }
    [Serializable]
    public class BvhNode
    {
        public AABB BvhAABB;
        public BvhNode Left;
        public BvhNode Right;
        public int[] TriangleIndices;

        public bool IsLeaf => TriangleIndices != null;
    }
    //这个数据为了能够方便在CS中访问，做了扁平化处理，去掉多余的类，需要和Triangle数组搭配使用
    [System.Serializable]
    public struct BvhData
    {
        public float3 Min;
        public float3 Max;

        public int LeftIdx;
        public int RightIdx;

        public int TriangleStartIdx;//-1 if not leaf 
        public int TrianglesCount;
        public bool IsLeaf => TriangleStartIdx >= 0;

    }
    //构造四面体和计算模拟用

    [Serializable]
    public struct Tet
    {
        public int4 index;
        public float volume;
    }
    [Serializable]
    public struct Edge : IEquatable<Edge>
    {
        public int2 index;

        public float length;
        public Edge(int x, int y)
        {
            index.x = x <= y ? x : y;
            index.y = x <= y ? y : x;
            length = 0;
        }
        public bool Equals(Edge other) => index.x == other.index.x && index.y == other.index.y;
        public override bool Equals(object obj) => obj is Edge other && Equals(other);
        public override int GetHashCode() => unchecked(index.x * 397) ^ index.y;
    }

    //对于每一个网格上的顶点，在初始化时预计算一个此结构，维护了该顶点最近的四面体编号，以及它在这个四面体内的重心坐标
    [Serializable]
    public struct VtxResult
    {
        public float4 bc;
        public int tetIdx;
    }
    [Serializable]
    public struct VertToTetLink
    {
        public int tetIndex;
        public int dataOffset;
    }
    [Serializable]
    public struct VertToEdgeLink
    {
        public int edgeIndex;
        public int dataOffset;
    }
    static class ShaderParams
    {
        //全局KernelID:生成阶段
        public static string KernelCheckPoint = "CheckPoint";
        public static string KernelCheckTet = "CheckTetCenter";
        public static string KernelGetMeshVtxArgs = "GetMeshVtxArgs";
        //全局KernelID:模拟阶段
        public static string KernelSetBufferData = "SetBufferData";
        public static string KernelPreSolve = "PreSolve";
        public static string KernelSolveHeights = "SolveHeights";
        public static string KernelSolveEdges = "SolveEdges";
        public static string KernelSolveVolumes = "SolveVolumes";
        public static string KernelMergeResults = "MergeResults";
        public static string KernelPostSolve = "PostSolve";
        public static string KernelApplyMeshVertex = "ApplyMeshVertex";
        //生成阶段变量
        public static int CellOriginArrayID = Shader.PropertyToID("cellOriginArray");
        public static int CellSizeVectorID = Shader.PropertyToID("cellSizeVector");
        public static int CellCountArrayID = Shader.PropertyToID("cellCountArray");
        //生成阶段BufferID
        public static int TriangleBufID = Shader.PropertyToID("triangleBuffer");
        public static int BvhBufID = Shader.PropertyToID("bvhBuffer");
        public static int PointsBufID = Shader.PropertyToID("pointsBuffer");
        public static int PointResultBufID = Shader.PropertyToID("pointsResultBuffer");
        public static int MinDistID = Shader.PropertyToID("minDist");
        public static int verticesBufID = Shader.PropertyToID("verticesBuffer");
        public static int TetsBufID = Shader.PropertyToID("tetsBuffer");//这个共用
        public static int TetsResultBufID = Shader.PropertyToID("tetResultBuffer");
        //运行阶段变量
        public static int MeshVtxStrideID = Shader.PropertyToID("meshVtxStride");
        public static int DeltaTimeID = Shader.PropertyToID("dt");
        // public static int SubStepsID = Shader.PropertyToID("subSteps");

        public static int TetVtxCountID = Shader.PropertyToID("tetVtxCount");
        public static int TetCountID = Shader.PropertyToID("tetCount");
        public static int EdgeCountID = Shader.PropertyToID("edgeCount");
        //运行阶段每个TetAsset内部维护的buffer
        public static int TetVTXOrginBufID = Shader.PropertyToID("tetVtxOriginBuffer");
        public static int EdgesBufID = Shader.PropertyToID("edgesBuffer");
        public static int MeshVtxBufID = Shader.PropertyToID("meshVtxBuffer");
        public static int MeshVtxArgsBufID = Shader.PropertyToID("meshVtxArgsBuffer");
        public static int VertsToEdgesBufID = Shader.PropertyToID("vertsToEdgesBuffer");
        public static int VertsToEdgesOffsetBufID = Shader.PropertyToID("vertsToEdgesOffsetBuffer");
        public static int VertsToTetsBufID = Shader.PropertyToID("vertsToTetsBuffer");
        public static int VertsToTetsOffsetBufID = Shader.PropertyToID("vertsToTetsOffsetBuffer");
        public static int CellsToTetsBufID = Shader.PropertyToID("cellsToTetsBuffer");
        public static int CellsToTetsOffsetBufID = Shader.PropertyToID("cellsToTetsOffsetBuffer");
        //每个对象自己维护
        public static int LocalToWorldMatBufID = Shader.PropertyToID("localToWorldMatBuffer");
        public static int WorldToLocalMatBufID = Shader.PropertyToID("worldToLocalMatBuffer");
        public static int EdgeComplianceID = Shader.PropertyToID("edgeCompliance");
        public static int VolumeComplianceID = Shader.PropertyToID("volumeCompliance");
        public static int InvMassBufID = Shader.PropertyToID("invMassBuffer");
        public static int EdgeSolveResultBufID = Shader.PropertyToID("edgeSolveResultBuffer");
        public static int VolumeSolveResultBufID = Shader.PropertyToID("volumeSolveResultBuffer");
        public static int TetVtxPosBufID = Shader.PropertyToID("tetVtxPosBuffer");
        public static int TetVtxVelocityBufID = Shader.PropertyToID("tetVtxVelocityBuffer");
        public static int TetVtxTempPosBufID = Shader.PropertyToID("tetVtxTempPosBuffer");

        public static int ObjectIndexID = Shader.PropertyToID("objectIndex");
    }
}
