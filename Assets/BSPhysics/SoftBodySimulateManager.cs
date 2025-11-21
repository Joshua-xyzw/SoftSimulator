using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor;
using UnityEditor.VersionControl;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.Rendering;
using UnityEngine.UI;
using Random = UnityEngine.Random;

namespace BSPhysics
{
    [DisallowMultipleComponent]
    public class SoftBodySimulateManager : MonoBehaviour
    {

        public static SoftBodySimulateManager Instance
        {
            get
            {
                // 编辑器模式下特殊处理
#if UNITY_EDITOR
                if (!Application.isPlaying)
                {
                    // 在场景中查找已存在的实例
                    _instance = FindFirstObjectByType<SoftBodySimulateManager>();
                    if (_instance == null)
                    {
                        // 创建隐藏的编辑器专用对象
                        var go = new GameObject("SimulateManager");
                        _instance = go.AddComponent<SoftBodySimulateManager>();
                        go.hideFlags = HideFlags.DontSave; // 防止保存到场景
                    }
                    return _instance;
                }
#endif
                return _instance;
            }
        }
        private static SoftBodySimulateManager _instance;
        private SoftBodySimulateManager() { }
        //全局设置
        //模拟的时间间隔
        [SerializeField]
        private ComputeShader SimulateCS;

        //单次模拟的子步骤划分
        [Range(10, 200)]
        public int SubSteps = 10;

        public float EdgeCompliance = 1e-12f;
        public float VolumeCompliance = 1e-12f;
        private static int OBJECT_SIZE = 256;
        private int currentObjectCount = 0;

        //内部维护的资源索引，指向所有被激活的的TetAsset，第二位为计数
        // private Dictionary<TetAsset, int> _TetAssetsTable = new Dictionary<TetAsset, int>();
        TetAsset tetAsset;
        private bool isDataReady = false;
        private float4x4[] localToWorldMatrixArr = new float4x4[OBJECT_SIZE];
        private float4x4[] worldToLocalMatrixArr = new float4x4[OBJECT_SIZE];
        private List<ComputeBuffer> localBufferList = new List<ComputeBuffer>(OBJECT_SIZE);
        private List<GraphicsBuffer> vertexBufferList = new List<GraphicsBuffer>(OBJECT_SIZE);

        private Material lineMaterial;
        private CommandBuffer drawLineCMD;
        private int kernelSetBufferdataID;
        private int kernelPreSolveID;
        private int kernelSolveHeightsID;
        private int kernelSolveEdgesID;
        private int kernelSolveVolumeID;
        private int kernelMergeResultsID;
        private int kernelPostSolveID;
        private int KernelApplyMeshVertexID;
        private uint countThreadGroups;
        int tetVtxNum;
        int tetNum;
        int edgeNum;
        int meshVtxCount;
        ComputeBuffer tetVtxPosBuffer;
        ComputeBuffer tetVtxVelocityBuffer;
        ComputeBuffer tetVtxTempPosBuffer;
        ComputeBuffer edgeSolveResultBuffer;
        ComputeBuffer volumeSolveResultBuffer;
        ComputeBuffer localToWorldMatBuffer;
        ComputeBuffer worldToLocalMatBuffer;
        // GraphicsBuffer meshVtxArgsBuffer;
        private bool isExcuted = false;
        private int assetTetNum = 0;
        private Text numText;
        public void SetSimulateCS(ComputeShader cs)
        {
            SimulateCS = cs;
        }
        private void Awake()
        {
            if (_instance == null) _instance = this;
            numText = GameObject.Find("TotalTets").GetComponent<Text>();
        }
        void Start()
        {

        }

        // Update is called once per frame
        void LateUpdate()
        {
            if (isDataReady)
            {
                UpdateSimulate();
                // isExcuted = true;
            }
            // if (Input.GetKeyDown(KeyCode.G))
            // {
            //     GetData();
            // }
        }
        // public float4[] vetexPos;
        // public float4[] vetexVelocity;

        // public float4x4[] matrixs;
        // void GetData()
        // {
        //     vetexPos = new float4[tetVtxNum * OBJECT_SIZE];
        //     vetexVelocity = new float4[tetVtxNum * OBJECT_SIZE];
        //     matrixs = new float4x4[OBJECT_SIZE];
        //     tetVtxPosBuffer.GetData(vetexPos);
        //     tetVtxVelocityBuffer.GetData(vetexVelocity);
        //     localToWorldMatBuffer.GetData(matrixs);
        // }
        void OnDestroy()
        {
            ClearAllAssetBuffer();
        }
        void OnApplicationQuit()
        {
            ClearAllAssetBuffer();
        }
        public ComputeShader GetSimulateCS()
        {
            return SimulateCS;
        }
        // public void RegisterTet(TetAsset asset)
        // {
        //     if (_TetAssetsTable.TryGetValue(asset, out int count))
        //     {
        //         _TetAssetsTable[asset] = count + 1;
        //     }
        //     else
        //     {
        //         _TetAssetsTable.Add(asset, 0);
        //         CreateAssetBuffer(asset, SimulateCS);
        //     }
        // }
        // public void UnregisterTet(TetAsset asset)
        // {
        //     if (_TetAssetsTable.TryGetValue(asset, out int count))
        //     {
        //         if (count > 1)
        //         {
        //             _TetAssetsTable[asset] = count - 1;
        //         }
        //         else
        //         {
        //             _TetAssetsTable.Remove(asset);
        //             ReleaseAssetBuffer(asset, SimulateCS);
        //         }
        //     }
        //     else
        //     {
        //         Debug.LogError("TetAsset资源列表已清空仍然尝试移除,检查计数器逻辑");
        //     }
        // }
        // private void CreateAssetBuffer(TetAsset asset, ComputeShader CS)
        // {
        //     asset.CreateSimulateBuffer(CS);
        // }
        // private void ReleaseAssetBuffer(TetAsset asset, ComputeShader CS)
        // {
        //     asset.ReleaseAssetBuffer();
        // }
        private void ClearAllAssetBuffer()
        {
            // foreach (var asset in _TetAssetsTable)
            // {
            //     ReleaseAssetBuffer(asset.Key, SimulateCS);
            // }
            // Camera.main.RemoveAllCommandBuffers();
            // drawLineCMD?.Release();
            tetAsset?.ReleaseAssetBuffer();
            foreach (var buffer in localBufferList)
            {
                buffer?.Dispose();
                buffer?.Release();
            }
        }
        public int RegistNewTet(Transform transform, TetAsset asset, GraphicsBuffer vertexBuffer, int vertexStride)
        {
            //第一次注册需要初始化所有Buffer
            if (currentObjectCount == 0)
            {
                PrepareKernelID();
                tetAsset = asset;
                asset.CreateSimulateBuffer(SimulateCS);
                InitialComputeBuffer(asset, vertexStride);
                SetDrawSetCMD();
                assetTetNum = asset.tetras.Length;
            }
            if (currentObjectCount >= OBJECT_SIZE) return currentObjectCount;//数量超过256则暂时不再新注册
            //设置转换矩阵
            localToWorldMatrixArr[currentObjectCount] = transform.localToWorldMatrix;
            worldToLocalMatrixArr[currentObjectCount] = transform.worldToLocalMatrix;
            localToWorldMatBuffer.SetData(localToWorldMatrixArr, 0, 0, currentObjectCount + 1);
            worldToLocalMatBuffer.SetData(worldToLocalMatrixArr, 0, 0, currentObjectCount + 1);
            vertexBufferList.Add(vertexBuffer);
            currentObjectCount++;
            UpdateDrawTetCMD();
            UpdateTextNum();
            return currentObjectCount - 1;
        }
        private void PrepareKernelID()
        {
            kernelSetBufferdataID = SimulateCS.FindKernel(ShaderParams.KernelSetBufferData);

            kernelPreSolveID = SimulateCS.FindKernel(ShaderParams.KernelPreSolve);

            kernelSolveHeightsID = SimulateCS.FindKernel(ShaderParams.KernelSolveHeights);

            kernelSolveEdgesID = SimulateCS.FindKernel(ShaderParams.KernelSolveEdges);

            kernelSolveVolumeID = SimulateCS.FindKernel(ShaderParams.KernelSolveVolumes);

            kernelMergeResultsID = SimulateCS.FindKernel(ShaderParams.KernelMergeResults);

            kernelPostSolveID = SimulateCS.FindKernel(ShaderParams.KernelPostSolve);

            KernelApplyMeshVertexID = SimulateCS.FindKernel(ShaderParams.KernelApplyMeshVertex);
        }
        // 每次重新申请会导致数值丢失，不如一开始申请足够大显存，按256个对象申请
        private void InitialComputeBuffer(TetAsset asset, int vertexStride)
        {
            //获取基本参数
            tetVtxNum = asset.vertices.Length;
            tetNum = asset.tetras.Length;
            edgeNum = asset.edges.Length;
            meshVtxCount = asset.vtxResults.Length;

            //初始化所有计算中会用到的ComputeBuffer，以256数量来申请一步到位
            tetVtxPosBuffer = new ComputeBuffer(tetVtxNum * OBJECT_SIZE, Marshal.SizeOf<float4>());
            tetVtxVelocityBuffer = new ComputeBuffer(tetVtxNum * OBJECT_SIZE, Marshal.SizeOf<float4>());
            tetVtxTempPosBuffer = new ComputeBuffer(tetVtxNum * OBJECT_SIZE, Marshal.SizeOf<float4>());
            edgeSolveResultBuffer = new ComputeBuffer(edgeNum * 2 * OBJECT_SIZE, Marshal.SizeOf<float4>());
            volumeSolveResultBuffer = new ComputeBuffer(tetNum * 4 * OBJECT_SIZE, Marshal.SizeOf<float4>());
            localToWorldMatBuffer = new ComputeBuffer(OBJECT_SIZE, Marshal.SizeOf<float4x4>());
            worldToLocalMatBuffer = new ComputeBuffer(OBJECT_SIZE, Marshal.SizeOf<float4x4>());

            localBufferList.Add(tetVtxPosBuffer);
            localBufferList.Add(tetVtxVelocityBuffer);
            localBufferList.Add(tetVtxTempPosBuffer);
            localBufferList.Add(edgeSolveResultBuffer);
            localBufferList.Add(volumeSolveResultBuffer);
            localBufferList.Add(localToWorldMatBuffer);
            localBufferList.Add(worldToLocalMatBuffer);
            //输入数据绑定buffer和设置全局变量
            //在Update之前将所有要用到的buffer做好绑定,及无需逐帧更新的变量
            SimulateCS.SetInt(ShaderParams.MeshVtxStrideID, vertexStride);
            SimulateCS.SetInt(ShaderParams.TetVtxCountID, tetVtxNum);
            SimulateCS.SetInt(ShaderParams.EdgeCountID, edgeNum);
            SimulateCS.SetInt(ShaderParams.TetCountID, tetNum);
            SimulateCS.SetFloat(ShaderParams.EdgeComplianceID, EdgeCompliance);
            SimulateCS.SetFloat(ShaderParams.VolumeComplianceID, VolumeCompliance);

            SimulateCS.SetBuffer(kernelSetBufferdataID, ShaderParams.TetVtxPosBufID, tetVtxPosBuffer);

            SimulateCS.SetBuffer(kernelPreSolveID, ShaderParams.TetVtxPosBufID, tetVtxPosBuffer);//读初始位置
            SimulateCS.SetBuffer(kernelPreSolveID, ShaderParams.TetVtxVelocityBufID, tetVtxVelocityBuffer);//读初始速度
            SimulateCS.SetBuffer(kernelPreSolveID, ShaderParams.TetVtxTempPosBufID, tetVtxTempPosBuffer);//写入更新位
            SimulateCS.SetBuffer(kernelPreSolveID, ShaderParams.WorldToLocalMatBufID, worldToLocalMatBuffer);

            SimulateCS.SetBuffer(kernelSolveHeightsID, ShaderParams.TetVtxPosBufID, tetVtxPosBuffer);//读初始位置
            SimulateCS.SetBuffer(kernelSolveHeightsID, ShaderParams.TetVtxTempPosBufID, tetVtxTempPosBuffer);//写入执行约束后更新位置
            SimulateCS.SetBuffer(kernelSolveHeightsID, ShaderParams.LocalToWorldMatBufID, localToWorldMatBuffer);
            SimulateCS.SetBuffer(kernelSolveHeightsID, ShaderParams.WorldToLocalMatBufID, worldToLocalMatBuffer);

            SimulateCS.SetBuffer(kernelSolveEdgesID, ShaderParams.TetVtxTempPosBufID, tetVtxTempPosBuffer);//读取最新位置
            SimulateCS.SetBuffer(kernelSolveEdgesID, ShaderParams.EdgeSolveResultBufID, edgeSolveResultBuffer);//写入边约束结果

            SimulateCS.SetBuffer(kernelSolveVolumeID, ShaderParams.TetVtxTempPosBufID, tetVtxTempPosBuffer);//读取最新位置
            SimulateCS.SetBuffer(kernelSolveVolumeID, ShaderParams.VolumeSolveResultBufID, volumeSolveResultBuffer);//写入体积约束结果

            SimulateCS.SetBuffer(kernelMergeResultsID, ShaderParams.TetVtxTempPosBufID, tetVtxTempPosBuffer);//读写的位置
            SimulateCS.SetBuffer(kernelMergeResultsID, ShaderParams.EdgeSolveResultBufID, edgeSolveResultBuffer);
            SimulateCS.SetBuffer(kernelMergeResultsID, ShaderParams.VolumeSolveResultBufID, volumeSolveResultBuffer);

            SimulateCS.SetBuffer(kernelPostSolveID, ShaderParams.TetVtxTempPosBufID, tetVtxTempPosBuffer);
            SimulateCS.SetBuffer(kernelPostSolveID, ShaderParams.TetVtxPosBufID, tetVtxPosBuffer);
            SimulateCS.SetBuffer(kernelPostSolveID, ShaderParams.TetVtxVelocityBufID, tetVtxVelocityBuffer);

            SimulateCS.SetBuffer(KernelApplyMeshVertexID, ShaderParams.TetVtxPosBufID, tetVtxPosBuffer);
            //需要预执行一次四面体顶点位置复制填充
            SimulateCS.GetKernelThreadGroupSizes(kernelSetBufferdataID, out countThreadGroups, out var _, out var _);
            SimulateCS.Dispatch(kernelSetBufferdataID, (int)math.ceil(tetVtxNum / (float)countThreadGroups), OBJECT_SIZE, 1);
            // var _request = AsyncGPUReadback.Request(tetVtxPosBuffer);
            // _request.WaitForCompletion();
            AsyncGPUReadback.WaitAllRequests();
            isDataReady = true;
        }
        //调用ComputeShader重新给各对象的四面体顶点的buffer
        private void UpdateSimulate()
        {
            SimulateCS.SetFloat(ShaderParams.DeltaTimeID, Time.fixedDeltaTime);
            SimulateCS.SetFloat(ShaderParams.EdgeComplianceID, EdgeCompliance);
            SimulateCS.SetFloat(ShaderParams.VolumeComplianceID, VolumeCompliance);
            localToWorldMatBuffer.SetData(localToWorldMatrixArr, 0, 0, currentObjectCount);
            worldToLocalMatBuffer.SetData(worldToLocalMatrixArr, 0, 0, currentObjectCount);

            SimulateCS.Dispatch(kernelPreSolveID, (int)math.ceil(tetVtxNum / (float)countThreadGroups), currentObjectCount, 1);
            for (int i = 0; i < SubSteps; i++)
            {
                SimulateCS.Dispatch(kernelSolveHeightsID, (int)math.ceil(tetVtxNum / (float)countThreadGroups), currentObjectCount, 1);
                SimulateCS.Dispatch(kernelSolveEdgesID, (int)math.ceil(edgeNum / (float)countThreadGroups), currentObjectCount, 1);
                SimulateCS.Dispatch(kernelSolveVolumeID, (int)math.ceil(tetNum / (float)countThreadGroups), currentObjectCount, 1);
                SimulateCS.Dispatch(kernelMergeResultsID, (int)math.ceil(tetVtxNum / (float)countThreadGroups), currentObjectCount, 1);
            }
            SimulateCS.Dispatch(kernelPostSolveID, (int)math.ceil(tetVtxNum / (float)countThreadGroups), currentObjectCount, 1);
            //为了省去CPU回读的时间这里直接操作显存，但是显存不连续，computeShader中又限制指针操作，所以只能逐个赋值调用,注意因为要一个一个绘制但是顶点位置又在一起这里要更新是第几个buffer来找顶点位置
            for (int i = 0; i < currentObjectCount; i++)
            {
                SimulateCS.SetInt(ShaderParams.ObjectIndexID, i);
                SimulateCS.SetBuffer(KernelApplyMeshVertexID, ShaderParams.MeshVtxBufID, vertexBufferList[i]);
                SimulateCS.Dispatch(KernelApplyMeshVertexID, (int)math.ceil(meshVtxCount / (float)countThreadGroups), 1, 1);
            }
        }
        private void SetDrawSetCMD()
        {
            // 创建Line Shader材质
            lineMaterial = new Material(Shader.Find("Hidden/LineShader"));
            lineMaterial.SetColor("_Color", Color.yellow);
            lineMaterial.SetBuffer("_Edges", tetAsset.GetEdgesBuffer());
            lineMaterial.SetInt(ShaderParams.EdgeCountID, edgeNum);
            lineMaterial.SetInt(ShaderParams.TetVtxCountID, tetVtxNum);
            //设置动态绘制的cmd,CMD不应每帧创建，可以可以每次清除
            drawLineCMD = new CommandBuffer();
            drawLineCMD.name = "DrawTetEdges";



        }
        private void UpdateDrawTetCMD()
        {
            Camera.main.RemoveCommandBuffer(CameraEvent.AfterForwardOpaque, drawLineCMD);
            drawLineCMD.Clear();
            drawLineCMD.SetGlobalBuffer("_Positions", tetVtxPosBuffer);
            drawLineCMD.SetGlobalBuffer("_LocalToWorldMatrixs", localToWorldMatBuffer);
            Camera.main.AddCommandBuffer(CameraEvent.AfterForwardOpaque, drawLineCMD);
            drawLineCMD.DrawProcedural(Matrix4x4.identity, lineMaterial, 0, MeshTopology.Lines, 2, edgeNum * currentObjectCount);
            //如果已经使用Camera.main.AddCommandBuffer显示的绑定到了相机，则无需显示要求执行执行，会在相机的相应事件时自动调用
            // Graphics.ExecuteCommandBuffer(drawLineCMD);
        }
        void UpdateTextNum()
        {
            numText.text = (currentObjectCount*assetTetNum).ToString();
        }
    }
}
