using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor;
using UnityEditor.VersionControl;
using UnityEngine;
using UnityEngine.Rendering;

namespace BSPhysics
{
    [RequireComponent(typeof(MeshFilter))]
    [RequireComponent(typeof(MeshRenderer))]
    public class TetBehaviour : MonoBehaviour
    {
        public TetAsset TetAsset;
        public bool DrawTet = true;
        // public float EdgeCompliance = 100f;
        // public float VolumeCompliance = 100f;
        // private float[] invMassArray;

        private Mesh simulateMesh;
        [SerializeField]
        private int objectIndex;
        private ComputeShader TetSimulateCS;

        private GraphicsBuffer meshVertexBuffer;
        // private ComputeBuffer meshVtxArgsBuffer;
        // // private ComputeBuffer tetVtxPosBuffer;
        // // private ComputeBuffer tetVtxVelocityBuffer;

        // private ComputeBuffer tetVtxTempPosBuffer;
        // // private ComputeBuffer invMassBuffer;
        // private ComputeBuffer edgeSolveResultBuffer;
        // private ComputeBuffer volumeSolveResultBuffer;

        private List<ComputeBuffer> localBufferList = new List<ComputeBuffer>(7);

        // private CommandBuffer drawLineCMD;
        // private Material lineMaterial;
        private MeshRenderer meshRenderer;

        // private int tetVtxNum;
        // private int tetNum;
        // private int edgeNum;
        // private int meshVtxCount;

        // private uint CountThreadGroups;//每个kernel的线程组数量，因为每个在CS里都是同一个常熟所以没必要反复获取

        // private int kernelPreSolveID;
        // private int kernelSolveHeightsID;
        // private int kernelSolveEdgesID;
        // private int kernelSolveVolumeID;
        // private int kernelMergeResultsID;
        // private int kernelPostSolveID;
        // private int KernelApplyMeshVertexID;
        [SerializeField, HideInInspector] int _lastHash;

        // // //调试用的一些buffer
        // public float4[] meshVtxPos;
        // public float4[] tetVtxPos;
        // public VtxResult[] vtxResults;
        // public float4[] edgeSolveResult;
        // public float4[] volumeSolveResult;

        private void OnDrawGizmosSelected()
        {
            if (TetAsset == null) return;
            if (DrawTet && !Application.isPlaying)
            {
                TetAsset.DrawGizmo(transform);
            }
        }
        int GetTargetHash()
        {
            var sharedMesh = GetComponent<MeshFilter>().sharedMesh;
            return (TetAsset.GetInstanceID() * 397) ^ sharedMesh.GetInstanceID();
            // return  sharedMesh.GetInstanceID();
        }
#if UNITY_EDITOR
        //仅在编辑器下,如果引用的Asset发生变化会触发顶点在四面体中重心坐标的重新计算
        private void OnValidate()
        {
            if (!Application.isPlaying&&TetAsset!=null)
            {
                int newHash = GetTargetHash();
                if (newHash != _lastHash)
                {
                    _lastHash = newHash;
                    SetMeshArgs(TetAsset);
                }
            }

        }
#endif

        //使用ComputeShader计算模型网格各顶点的重心坐标
        public void SetMeshArgs(TetAsset asset)
        {
            meshRenderer = GetComponent<MeshRenderer>();
            var sharedsMesh = GetComponent<MeshFilter>().sharedMesh;
            TetSimulateCS = SoftBodySimulateManager.Instance.GetSimulateCS();
            if (meshRenderer == null || TetSimulateCS == null || TetAsset == null || sharedsMesh == null) return;
            var hash = GetTargetHash();
            TetAsset.CalculateMeshArgsInCS(TetSimulateCS, sharedsMesh, hash);
            // Debug.Log("更新顶点绑定关系");
        }
        void Start()
        {
            GetMeshRegist();
            // if (DrawTet != lastDrawtet)
            // {
            //     SetTetDraw(DrawTet);
            //     lastDrawtet = DrawTet;
            // }
            // if (EnableSimulate != lastEnableSim)
            // {
            //     SetSimulate(EnableSimulate);
            //     lastEnableSim = EnableSimulate;
            // }
        }
        void Update()
        {
            // // Getdata();
            // if (DrawTet != lastDrawtet)
            // {
            //     SetTetDraw(DrawTet);
            //     lastDrawtet = DrawTet;
            // }
            // if (EnableSimulate != lastEnableSim)
            // {
            //     SetSimulate(EnableSimulate);
            //     lastEnableSim = EnableSimulate;
            // }
        }
        private void GetMeshRegist()
        {
            simulateMesh = GetComponent<MeshFilter>().mesh;
            if (TetAsset == null || simulateMesh == null) return;
            simulateMesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
            var meshStride = simulateMesh.GetVertexBufferStride(0);
            meshVertexBuffer = simulateMesh.GetVertexBuffer(0);
            objectIndex = SoftBodySimulateManager.Instance.RegistNewTet(this.transform,TetAsset, meshVertexBuffer, meshStride);
        }
        // private void SetTetDraw(bool isDraw)
        // {
        //     if (isDraw) DrawCMDBuffer();
        //     else drawLineCMD.Clear();
        // }
        // private void DrawCMDBuffer()
        // {
        //     //设置动态绘制的cmd
        //     // 创建Line Shader材质
        //     if (meshRenderer.enabled)
        //     {
        //         lineMaterial = new Material(Shader.Find("Hidden/LineShader"));
        //         lineMaterial.SetColor("_Color", Color.yellow);
        //         drawLineCMD = new CommandBuffer();
        //         drawLineCMD.name = "DrawTetEdges";
        //         drawLineCMD.SetGlobalBuffer("_Positions", tetVtxPosBuffer);
        //         drawLineCMD.SetGlobalBuffer("_Edges", TetAsset.GetEdgesBuffer());
        //         Camera.main.AddCommandBuffer(CameraEvent.AfterForwardOpaque, drawLineCMD);
        //         drawLineCMD.DrawProcedural(transform.localToWorldMatrix, lineMaterial, 0, MeshTopology.Lines, 2, TetAsset.edges.Length);
        //     }
        // }
        // private void DispatchPreSolve(int iter)
        // {
        //     if (TetSimulateCS == null || simulateMesh == null) return;
        //     TetSimulateCS.Dispatch(kernelPreSolveID, (int)math.ceil(tetVtxNum / (float)CountThreadGroups), 1, 1);
        // }
        // private void DispatchConstrains(int iter)
        // {
        //     if (TetSimulateCS == null || simulateMesh == null) return;
        //     //这里都是要逐帧更新的参数
        //     UnityEngine.Matrix4x4 localToWorldMat = transform.localToWorldMatrix;
        //     UnityEngine.Matrix4x4 worldToLocalMat = transform.worldToLocalMatrix;

        //     // TetSimulateCS.SetMatrix(ShaderParams.LocalToWorldMatID, localToWorldMat);
        //     // TetSimulateCS.SetMatrix(ShaderParams.WorldToLocalMatID, worldToLocalMat);
        //     TetSimulateCS.SetFloat(ShaderParams.EdgeComplianceID, EdgeCompliance);
        //     TetSimulateCS.SetFloat(ShaderParams.VolumeComplianceID, VolumeCompliance);
        //     for (int i = 0; i < iter; i++)
        //     {
        //         TetSimulateCS.Dispatch(kernelSolveHeightsID, (int)math.ceil(tetVtxNum / (float)CountThreadGroups), 1, 1);
        //         TetSimulateCS.Dispatch(kernelSolveEdgesID, (int)math.ceil(edgeNum / (float)CountThreadGroups), 1, 1);
        //         TetSimulateCS.Dispatch(kernelSolveVolumeID, (int)math.ceil(tetNum / (float)CountThreadGroups), 1, 1);
        //         TetSimulateCS.Dispatch(kernelMergeResultsID, (int)math.ceil(tetVtxNum / (float)CountThreadGroups), 1, 1);
        //     }
        //     // GraphicsFence fence = Graphics.CreateGraphicsFence(GraphicsFenceType.AsyncQueueSynchronisation,SynchronisationStageFlags.ComputeProcessing)
        // }
        // private void DispatchPostSolve(int iter)
        // {
        //     if (TetSimulateCS == null || simulateMesh == null) return;


        //     TetSimulateCS.Dispatch(kernelPostSolveID, (int)math.ceil(tetVtxNum / (float)CountThreadGroups), 1, 1);
        //     TetSimulateCS.Dispatch(KernelApplyMeshVertexID, (int)math.ceil(meshVtxCount / (float)CountThreadGroups), 1, 1);
        // }
        private void OnDestroy()
        {
            // RemoveDelegate();
            ClearBuffer();
        }
        private void OnApplicationQuit()
        {
            // RemoveDelegate();
            ClearBuffer();
        }
        // private void SetSimulate(bool isSimulate)
        // {
        //     if (isSimulate)
        //     {
        //         BindDelegate();
        //     }
        //     else
        //     {
        //         RemoveDelegate();
        //     }
        // }

        void ClearBuffer()
        {
            foreach (var buffer in localBufferList)
            {
                ClearSpecificBuffer(buffer);
            }
            meshVertexBuffer?.Dispose();
            meshVertexBuffer?.Release();
        }
        //清除computebuffer并将引用清空
        void ClearSpecificBuffer(ComputeBuffer buffer)
        {
            buffer?.Dispose();
            buffer?.Release();
        }
        // // //调试用
        // void Getdata()
        // {
        //     tetVtxTempPosBuffer.GetData(tetVtxPos);
        //     meshVtxArgsBuffer.GetData(vtxResults);
        //     edgeSolveResultBuffer.GetData(edgeSolveResult);
        //     volumeSolveResultBuffer.GetData(volumeSolveResult);
        // }
    }
}
