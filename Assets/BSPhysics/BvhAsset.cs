using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using System.Runtime.InteropServices;

namespace BSPhysics
{
    [System.Serializable]
    public class BVHAsset : ScriptableObject
    {

        public Triangle[] Triangles;
        //[FormerlySerializedAs("bvhDatas")]
        public BvhData[] BvhDatas;
        private ComputeBuffer triangleBuffer;
        private ComputeBuffer bvhBuffer;

        public (ComputeBuffer, ComputeBuffer) CreateComputeBuffers()
        {
            triangleBuffer = new ComputeBuffer( Triangles.Length, Marshal.SizeOf<Triangle>());
            triangleBuffer.SetData(Triangles);
            bvhBuffer = new ComputeBuffer(BvhDatas.Length, Marshal.SizeOf<BvhData>());
            bvhBuffer.SetData(BvhDatas);
            return (triangleBuffer,bvhBuffer);
        }
        public void ClearComputeBuffers()
        {
            triangleBuffer?.Dispose();
            triangleBuffer?.Release();
            bvhBuffer?.Dispose();
            bvhBuffer?.Release();
        }
        public void DrawGizmo(int gizmoDepth, bool gizmoLeafNodeOnly)
        {
            if (BvhDatas != null && Triangles != null)
            {
                DrawBvhGizmo(0, gizmoDepth, gizmoLeafNodeOnly);
            }
        }
        public void DrawBvhGizmo(int idx, int gizmoDepth, bool gizmoLeafNodeOnly, int recuriseCount = 0)
        {
            if (idx < 0 || BvhDatas.Length < idx) return;
            var data = BvhDatas[idx];
            if (gizmoDepth == recuriseCount)
            {
                //当前层为叶子节点则绘制所有三角形
                if (data.IsLeaf)
                {
                    Gizmos.color = Color.red;
                    for (int i = 0; i < data.TrianglesCount; i++)
                    {
                        var tri = Triangles[data.TriangleStartIdx + i];
                        Gizmos.DrawLine(tri.Pos0, tri.Pos1);
                        Gizmos.DrawLine(tri.Pos1, tri.Pos2);
                        Gizmos.DrawLine(tri.Pos2, tri.Pos0);
                    }
                }
                //若当前层为叶子节点或者指定全部显示，则绘制方形包围盒
                if (data.IsLeaf || !gizmoLeafNodeOnly)
                {
                    var aabb = new AABB(data.Min, data.Max);
                    Gizmos.color = data.IsLeaf ? Color.cyan : Color.green;
                    Gizmos.DrawWireCube(aabb.Center, aabb.Size);
                }
            }
            else if (!data.IsLeaf)
            {
                DrawBvhGizmo(data.LeftIdx, gizmoDepth, gizmoLeafNodeOnly, recuriseCount + 1);
                DrawBvhGizmo(data.RightIdx, gizmoDepth, gizmoLeafNodeOnly, recuriseCount + 1);
            }
        }
    }
}
