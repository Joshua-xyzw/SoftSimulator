using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Animations;

namespace BSPhysics
{
    public static class BvhBuilder
    {
        private static float3 ONE = new float3(1, 1, 1);
        public static (Triangle[], BvhData[]) BuildBVH(float3[] vertices, int[] triangleInx, int SplitCount)
        {
            var triangles = CreateTriangles(vertices, triangleInx);
            var bvhRoot = CreateBvh(triangles, SplitCount);
            var (bvhDataList, triangleIndexes) = CreateBvhData(bvhRoot);
            var sortedTriangles = triangleIndexes.Select(i => triangles[i]);
            return (sortedTriangles.ToArray(), bvhDataList.ToArray());
        }
        public static Triangle[] CreateTriangles(float3[] vertices, int[] triangleInx)
        {
            return Enumerable.Range(0, triangleInx.Length / 3).Select(i =>
            {
                return new Triangle()
                {
                    Pos0 = vertices[triangleInx[i * 3]],
                    Pos1 = vertices[triangleInx[i * 3 + 1]],
                    Pos2 = vertices[triangleInx[i * 3 + 2]],
                    normal = math.normalize(math.cross(vertices[triangleInx[i * 3 + 1]] - vertices[triangleInx[i * 3]],
                     vertices[triangleInx[i * 3 + 2]] - vertices[triangleInx[i * 3 + 1]]))
                };
            }).ToArray();
        }
        public static BvhNode CreateBvh(Triangle[] triangles, int splitCount)
        {
            BvhNode rootNode = new BvhNode();
            //申明并初始化triangleDatas，记得使用Native容器需要手动释放内存
            NativeArray<TriangleData> triangleDatas = new NativeArray<TriangleData>(triangles.Length, Allocator.Temp);
            for (int i = 0; i < triangles.Length; i++)
            {

                float3 min = new float3(math.min(triangles[i].Pos0, math.min(triangles[i].Pos1, triangles[i].Pos2)));
                float3 max = new float3(math.max(triangles[i].Pos0, math.max(triangles[i].Pos1, triangles[i].Pos2)));
                triangleDatas[i] = new TriangleData()
                {
                    TriangleAABB = new AABB(min, max),
                    TriangleIndex = i
                };
            }
            rootNode = CreateBvhRecrusive(triangleDatas, splitCount);

            triangleDatas.Dispose();
            return rootNode;
        }
        //构造函数，首先利用SAH选中轴，排序，递归分解
        private static BvhNode CreateBvhRecrusive(NativeSlice<TriangleData> triangleAABBArray, int splitCount, int recursiveCount = 0)
        {
            float bestSplit = 0f;
            int bestAxis = -1;//x->0,y->1,z->2, 
            if (triangleAABBArray.Length >= 4)
            {
                var (totalAABB, minCost) = CalculateAABBandSAH(triangleAABBArray);
                var size = totalAABB.Size;
                //申明缓存空间，后面的遍历都复用此空间
                NativeArray<TriangleData> leftBuf = new NativeArray<TriangleData>(triangleAABBArray.Length, Allocator.Temp);
                NativeArray<TriangleData> rightBuf = new NativeArray<TriangleData>(triangleAABBArray.Length, Allocator.Temp);
                for (int axis = 0; axis < 3; axis++)
                {
                    if (size[axis] < 0.001) continue;
                    int bucketCount = splitCount / (recursiveCount + 1);//动态桶数，越下层桶越少
                    float step = size[axis] / bucketCount;
                    float stepStart = totalAABB.Min[axis] + step;
                    float stepEnd = totalAABB.Max[axis] - step;
                    for (var testSplit = stepStart; testSplit <= stepEnd; testSplit += step)
                    {
                        var (left, right) = SplitLR(triangleAABBArray, ref leftBuf, ref rightBuf, axis, testSplit);
                        if (left.Length <= 1 || right.Length <= 1) continue;
                        var (_, leftCost) = CalculateAABBandSAH(left);
                        var (_, rightCost) = CalculateAABBandSAH(right);
                        var totalCost = leftCost + rightCost;
                        if (totalCost < minCost)
                        {
                            minCost = totalCost;
                            bestSplit = testSplit;
                            bestAxis = axis;
                        }

                    }
                }
                leftBuf.Dispose();
                rightBuf.Dispose();
            }
            BvhNode ret;
            if (bestAxis < 0)
            {
                ret = CreateNewBvhLeaf(triangleAABBArray);
            }
            else
            {
                var leftBuf = new NativeArray<TriangleData>(triangleAABBArray.Length, Allocator.Temp);
                var rightBuf = new NativeArray<TriangleData>(triangleAABBArray.Length, Allocator.Temp);

                var (left, right) = SplitLR(triangleAABBArray,ref leftBuf, ref rightBuf, bestAxis, bestSplit);

                var leftNode = CreateBvhRecrusive(left, splitCount, recursiveCount + 1);
                var rightNode = CreateBvhRecrusive(right, splitCount, recursiveCount + 1);

                var aabb = UnionAABB(leftNode.BvhAABB,rightNode.BvhAABB);
                

                ret = new BvhNode()
                {
                    BvhAABB = aabb,
                    Left = leftNode,
                    Right = rightNode
                };

                rightBuf.Dispose();
                leftBuf.Dispose();
            }
            return ret;
        }
        private static AABB UnionAABB(AABB a,AABB b)
        {
            return new AABB(math.min(a.Min,b.Min),math.max(a.Max,b.Max));
        }
        //这里求AABB符合二元结合律，其实可视为并行规约(Reduce)操作，可以使用CS进一步优化,这里是目前速度的主要瓶颈
        private static AABB CalculateAABB(NativeSlice<TriangleData> triangleAABBArray)
        {
            AABB aabb = new AABB(ONE * float.MaxValue,ONE * float.MinValue);
            for (int i = 0; i < triangleAABBArray.Length; i++)
            {
                aabb = UnionAABB(aabb,triangleAABBArray[i].TriangleAABB);
            }
            return aabb;
        }

        private static (AABB, float) CalculateAABBandSAH(NativeSlice<TriangleData> triangleAABBArray)
        {
            AABB aabb = CalculateAABB(triangleAABBArray);
            var size = aabb.Size;
            float area = (size.x * size.y + size.x * size.z + size.y * size.z) * triangleAABBArray.Length;
            return (aabb, area);
        }

        private static (NativeSlice<TriangleData> left, NativeSlice<TriangleData> right) SplitLR(NativeSlice<TriangleData> triangleAABBArray, ref NativeArray<TriangleData> leftBuf, ref NativeArray<TriangleData> rightBuf, int aixs, float split)
        {
            int leftCount = 0;
            int rightCount = 0;
            for (int i = 0; i < triangleAABBArray.Length; i++)
            {
                if (triangleAABBArray[i].TriangleAABB.Center[aixs] < split)
                {
                    leftBuf[leftCount++] = triangleAABBArray[i];
                }
                else
                {
                    rightBuf[rightCount++] = triangleAABBArray[i];
                }
            }
            return (leftBuf.Slice(0, leftCount), rightBuf.Slice(0, rightCount));
        }
        private static BvhNode CreateNewBvhLeaf(NativeSlice<TriangleData> triangleAABBArray)
        {
            return new BvhNode()
            {
                BvhAABB = CalculateAABB(triangleAABBArray),
                TriangleIndices = triangleAABBArray.Select(x => x.TriangleIndex).ToArray(),
            };
        }
        private static (List<BvhData>, List<int>) CreateBvhData(BvhNode root)
        {
            var datas = new List<BvhData>();
            var triangleIndexes = new List<int>();

            CreatteBvhDatasRecursive(root, datas, triangleIndexes);

            return (datas, triangleIndexes);
        }
        //使用深度优先遍历整个BVH树，依次插入到List中，按同样顺序构造三角面索引
        private static void CreatteBvhDatasRecursive(BvhNode node, List<BvhData> datas, List<int> triangleIndexes)
        {
            var data = new BvhData()
            {
                Min = node.BvhAABB.Min,
                Max = node.BvhAABB.Max,
                LeftIdx = -1,
                RightIdx = -1,
                TriangleStartIdx = -1,
                TrianglesCount = 0
            };
            if (node.IsLeaf)
            {
                var idx = triangleIndexes.Count;
                triangleIndexes.AddRange(node.TriangleIndices);
                data.TriangleStartIdx = idx;
                data.TrianglesCount = node.TriangleIndices.Length;
                datas.Add(data);
            }
            else
            {
                data.TriangleStartIdx = -1;
                var idx = datas.Count;
                datas.Add(default);//先占位再递归
                data.LeftIdx = datas.Count;
                CreatteBvhDatasRecursive(node.Left, datas, triangleIndexes);
                data.RightIdx = datas.Count;
                CreatteBvhDatasRecursive(node.Right, datas, triangleIndexes);
                datas[idx] = data;
            }
        }
    }

}
