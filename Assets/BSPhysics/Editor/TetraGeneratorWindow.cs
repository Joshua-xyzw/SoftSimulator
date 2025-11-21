using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System;
using System.IO;
using System.Linq;
using Unity.Mathematics;
using Unity.VisualScripting;


namespace BSPhysics.Editor
{
    public class TetraGeneratorWindow : EditorWindow
    {
        public MeshFilter ObjectMesh;
        public int SplitCount = 64;
        public int MaxBVHLeaf = 4;

        public ComputeShader CheckPointCS;

        public ComputeShader SimulateCS;

        public bool RemoveNearVertex = false;
        public float minVertexDist = 0.01f;
        public int TetraResolution = 6;
        public int MinQualityExp = -4;

        public float3 GridCellSize = new float3(0.5f, 0.5f, 0.5f);

        string lastPath;

        private static bool isRunning = false;

        [MenuItem("Window/OpenTetraGenerator")]
        static void Init()
        {
            var window = GetWindowWithRect<TetraGeneratorWindow>(new Rect(0f, 0f, 400f, 130f));
            window.Show();
        }
        void OnGUI()
        {
            GUILayout.Space(16f);
            GUIContent meshLabel = new GUIContent("MeshFilter", "选中你需要生成四面体的MeshFilter");
            ObjectMesh = EditorGUILayout.ObjectField(meshLabel, ObjectMesh, typeof(MeshFilter), true) as MeshFilter;
            GUILayout.Label("BVH参数");
            GUIContent splitLabel = new GUIContent("SplitCount", "BVH分割桶数");
            SplitCount = EditorGUILayout.IntSlider(splitLabel, SplitCount, 32, 128);
            GUIContent leafLabel = new GUIContent("MaxLeaf", "BVH最大叶子数");
            MaxBVHLeaf = EditorGUILayout.IntSlider(leafLabel, MaxBVHLeaf, 1, 4);
            GUILayout.Label("Tetra参数");
            GUIContent checkPointCSLabel = new GUIContent("CheckPointCS", "构建四面体时中所用的ComputeShader");
            CheckPointCS = EditorGUILayout.ObjectField(checkPointCSLabel, CheckPointCS, typeof(ComputeShader), true) as ComputeShader;
            GUIContent simulateCSLabel = new GUIContent("SimulateCS", "四面体模拟计算所用的ComputeShader");
            SimulateCS = EditorGUILayout.ObjectField(simulateCSLabel, SimulateCS, typeof(ComputeShader), true) as ComputeShader;
            GUIContent RemoveVertexLabel = new GUIContent("移除过近顶点", "是否允许移除构造四面体时移除过近顶点");
            RemoveNearVertex = EditorGUILayout.Toggle(RemoveVertexLabel, RemoveNearVertex);
            if (RemoveNearVertex)
            {
                GUIContent vertexMinDistLabel = new GUIContent("最小顶点距离", "构造四面体时允许的最小顶点距离,增加此值可显著减少四面体数量,但会导致细节丢失");
                minVertexDist = EditorGUILayout.FloatField(vertexMinDistLabel, minVertexDist);
            }
            GUIContent resolutionLabel = new GUIContent("分割段数", "构建四面体时对AABB分段分割添加中间辅助点,这里是每个方向的分割段数");
            TetraResolution = EditorGUILayout.IntSlider(resolutionLabel, TetraResolution, 1, 16);
            GUIContent qualityLabel = new GUIContent("四面体质量", "保留的四面体被允许的最低质量(10的指数)");
            MinQualityExp = EditorGUILayout.IntSlider(qualityLabel, MinQualityExp, -4, 0);
            GUIContent GridCellGridLabel = new GUIContent("每个空间网格尺寸", "空间加速网格尺寸,请参考模型实际大小设置");
            GridCellSize = EditorGUILayout.Vector3Field(GridCellGridLabel, GridCellSize);
            GUILayout.Space(16f);
            GUI.enabled = (ObjectMesh != null) && (CheckPointCS != null);
            if (GUILayout.Button("创建四面体"))
            {
                if (!ObjectMesh.sharedMesh.isReadable)
                {
                    Debug.LogError("检查模型文件是否开启允许读写");
                    return;
                }
                SoftBodySimulateManager.Instance?.SetSimulateCS(SimulateCS); ;
                var directory = "Assets";
                var defaultName = "TetraAsset";
                if (!string.IsNullOrEmpty(lastPath))
                {
                    directory = Path.GetDirectoryName(lastPath);
                    defaultName = Path.GetFileName(lastPath);
                }
                var path = EditorUtility.SaveFilePanel("Save Tetra Asset", directory, defaultName, "asset");
                if (!string.IsNullOrEmpty(path))
                {
                    lastPath = path;
                    var relativePath = "Assets" + path.Substring(Application.dataPath.Length);
                    //取出数据转为float3
                    float3[] vertices = ObjectMesh.sharedMesh.vertices.Select(x => new float3(x.x, x.y, x.z)).ToArray();
                    int[] triangleIdx = ObjectMesh.sharedMesh.triangles;
                    //构建BVH
                    var (triangles, bvhDatas) = BvhBuilder.BuildBVH(vertices, triangleIdx, SplitCount);
                    var bvhasset = CreateInstance<BVHAsset>();
                    bvhasset.Triangles = triangles;
                    bvhasset.BvhDatas = bvhDatas;
                    //构建四面体
                    var (tetVtxData, tetIdxData) = TetraBuilder.BuildTetra(CheckPointCS, vertices, bvhasset,RemoveNearVertex , minVertexDist,TetraResolution, math.pow(10f, MinQualityExp));
                    var tetAsset = AssetDatabase.LoadAssetAtPath<TetAsset>(relativePath);
                    if (tetAsset == null)
                    {
                        tetAsset = CreateInstance<TetAsset>();
                        AssetDatabase.CreateAsset(tetAsset, relativePath);

                    }
                    tetAsset.CellSize = math.abs(GridCellSize);
                    tetAsset.SetData(tetVtxData, tetIdxData);

#if UNITY_EDITOR
                    GC.Collect(); // 强制清理
#endif
                    isRunning = false;
                    var tetCom = ObjectMesh.gameObject.GetComponent<TetBehaviour>();
                    if (tetCom == null)
                    {
                        tetCom = ObjectMesh.gameObject.AddComponent<TetBehaviour>();
                    }
                    tetCom.TetAsset = tetAsset;
                    tetCom.SetMeshArgs(tetAsset);
                    EditorUtility.SetDirty(tetAsset);
                    AssetDatabase.SaveAssets();
                    EditorGUIUtility.PingObject(tetAsset);
                }
            }
        }
    }
}