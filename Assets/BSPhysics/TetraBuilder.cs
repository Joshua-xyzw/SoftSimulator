using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using System.Runtime.InteropServices;
using System.Linq;
using UnityEngine.Rendering;
using Unity.VisualScripting;
using Random = Unity.Mathematics.Random;
using System;
using UnityEngine.UIElements;
namespace BSPhysics
{
    public static class TetraBuilder
    {
        //如果生成四面体有交叉或者模拟有问题的话可以尝试降低此容差
        //但是射线法找冲突四面体时有一个判断t<FLOAT_EPSLON,这个容易引起冲突，如果容差太小容易导致顶点插入失败，那里可以单设一个容差
        private const float FLOAT_EPSLON = (float)1E-4;
        public static int[,] TETTRIS = new int[,] { { 0, 1, 2 }, { 0, 2, 3 }, { 1, 3, 2 }, { 0, 3, 1 } };

        //求四面体外界球球心
        //首先将四面体的P0即A点移动到原点，则BCD位置更新为bcd；根据四个点到外心距离构造四个等式，然后使用克莱姆法则用行列式求解
        public static float3 GetCircumCenter(float3 p0, float3 p1, float3 p2, float3 p3)
        {
            float3 b = p1 - p0;
            float3 c = p2 - p0;
            float3 d = p3 - p0;

            float det = math.dot(math.cross(c, b), d) * 2f;

            if (math.abs(det) < FLOAT_EPSLON) // 接近退化时返回重心
                return 0.25f * (p0 + p1 + p2 + p3);
            else
            {
                float3 v = math.lengthsq(b) * math.cross(d, c) + math.lengthsq(c) * math.cross(b, d) + math.lengthsq(d) * math.cross(c, b);
                v /= det;
                return p0 + v;
            }
        }
        //衡量四面体质量的算法
        //首先求出四面体各边的均方根rms，已知正四面体体积公式为v=1/12*sqrt(2)*pow(a,3)，a为棱长
        //这里直接用混合积求出体积，然后衡量其与用各边均方根代替棱长求出的正四面体WWWWWWWW体积的比例，以此比例作为四面体与正四面体的偏差
        static float TetQuality(float3 p0, float3 p1, float3 p2, float3 p3)
        {
            float3 d0 = p1 - p0;
            float3 d1 = p2 - p0;
            float3 d2 = p3 - p0;
            float3 d3 = p2 - p1;
            float3 d4 = p3 - p1;
            float3 d5 = p3 - p2;

            float rms =
             math.sqrt((
             math.lengthsq(d0) +
             math.lengthsq(d1) +
             math.lengthsq(d2) +
             math.lengthsq(d3) +
             math.lengthsq(d4) +
             math.lengthsq(d5)) / 6f);//均方根是先平方平均后再开方
            float s = 12 / math.sqrt(2);
            //这里算体积需要把顶点画出来看再确定顺序,否则容易出错
            float v = math.dot(d2, math.cross(d1, d0)) / 6;
            return (float)(v * s / math.pow(rms, 3));
        }
        //将输入变量转换为随机的正负极小值，大小与输入正相关
        static float RandomEps(ref Random random, float size)
        {
            float eps = size * FLOAT_EPSLON;
            return (float)random.NextDouble() * 2f * eps - eps;
            // return 0;
        }
        //将输入顶点按照某种小尺度偏移
        static float3 OffsetVetices(ref Random random, float3 pos, float size)
        {
            pos.x += RandomEps(ref random, size);
            pos.y += RandomEps(ref random, size);
            pos.z += RandomEps(ref random, size);
            return pos;
        }
        public struct TetraEdge
        {
            public int p0;
            public int p1;

            public int tet;

            public int face;//描述这条边对应新四面体的第几个邻居

            public TetraEdge(int point0, int point1, int tetraIndex, int faceIndex)
            {
                p0 = point0;
                p1 = point1;
                tet = tetraIndex;
                face = faceIndex;
            }
        }
        public static int CompareEdges(TetraEdge e0, TetraEdge e1)
        {
            if (e0.p0 < e1.p0 || (e0.p0 == e1.p0 && e0.p1 < e1.p1)) return -1;
            else return 1;
        }
        public static bool EqualEdges(TetraEdge e0, TetraEdge e1)
        {
            if (e0.p0 == e1.p0 && e0.p1 == e1.p1) return true;
            else return false;
        }
        public static float GetTetVolume(int4 tet, float4[] vertexDatas)
        {
            float3 p0 = vertexDatas[tet.x].xyz;
            float3 p1 = vertexDatas[tet.y].xyz;
            float3 p2 = vertexDatas[tet.z].xyz;
            float3 p3 = vertexDatas[tet.w].xyz;
            return GetTetVolume(p0, p1, p2, p3);
        }
        public static float GetTetVolume(float3 p0, float3 p1, float3 p2, float3 p3)
        {
            float3 d0 = p1 - p0;
            float3 d1 = p2 - p0;
            float3 d2 = p3 - p0;
            return math.dot(d2, math.cross(d1, d0)) / 6; ;
        }
        public static float4 GetBaryCentroidCoodinate(float3 p0, float3 p1, float3 p2, float3 p3, float3 p)
        {
            float v = GetTetVolume(p0, p1, p2, p3);
            float w0 = GetTetVolume(p, p1, p2, p3) / v;
            float w1 = GetTetVolume(p0, p, p2, p3) / v;
            float w2 = GetTetVolume(p0, p1, p, p3) / v;
            float w3 = 1 - w0 - w1 - w2;
            return new float4(w0, w1, w2, w3);
        }
        //需要返回顶点序列和四面体序列
        public static (float4[], int4[]) BuildTetra(ComputeShader CheckPointCS, float3[] vertices, BVHAsset bvhAsset, bool removeVtx, float minVertexDist, int resolution, float minQuality)
        {
            //初始化CS和bvh相关buffer
            var (triangleBuffer, bvhBuffer) = bvhAsset.CreateComputeBuffers();
            var random = new Random((uint)System.DateTime.Now.Ticks);
            var rootBvh = bvhAsset.BvhDatas[0];
            var size = rootBvh.Max - rootBvh.Min;
            float3 center = 0.5f * (rootBvh.Max + rootBvh.Min);
            var radius = 0.5f * math.length(size);
            var inputVertsCount = vertices.Length;
            inputVertsCount = math.clamp(inputVertsCount, 0, 65535);//限制最大顶点数量
            List<float4> tetVerts = new List<float4>(inputVertsCount + 4);
            if (removeVtx)
            {
                HashSet<int3> vertexHash = new HashSet<int3>();
                //使用哈希表剔除重复顶点,随机偏移后存入列表
                for (int i = 0; i < inputVertsCount; i++)
                {
                    int3 hashCell = (int3)(vertices[i] / minVertexDist);
                    if (vertexHash.Contains(hashCell)) continue;
                    vertexHash.Add(hashCell);
                    tetVerts.Add(new float4(OffsetVetices(ref random, vertices[i], radius), 1));
                }
                //划分内部采样点，CS中的minDist参数仅用于这里，防止将过于靠近表面的点加入Tet构建
                //因为要进CS考虑内存对齐扩充为float4
                //内部采样点也需要剔除重复
                List<float4> newVerts = new List<float4>((int)math.pow(resolution + 1, 3));
                float dim = math.max(math.max(size[0], size[1]), size[2]);
                float h = dim / resolution;
                for (int i = 0; i < (int)(size[0] / h) + 1; i++)
                {
                    float x = rootBvh.Min[0] + i * h;
                    for (int j = 0; j < (int)(size[1] / h) + 1; j++)
                    {
                        float y = rootBvh.Min[1] + j * h;
                        for (int k = 0; k < (int)(size[2] / h) + 1; k++)
                        {
                            // float z = rootBvh.Min[2] + k * h + RandomEps(ref random, radius);
                            float z = rootBvh.Min[2] + k * h;
                            var pos = new float3(x, y, z);
                            int3 posCell = (int3)(pos / minVertexDist);
                            if (vertexHash.Contains(posCell)) continue;
                            vertexHash.Add(posCell);
                            newVerts.Add(new float4(OffsetVetices(ref random, pos, radius), 1));
                        }
                    }
                }
                //如果实际有添加有效的内部采样点，则使用ComputeShader仅保留在模型内部的采样点
                if (newVerts.Count > 0)
                {
                    int kernelCheckPoint = CheckPointCS.FindKernel(ShaderParams.KernelCheckPoint);
                    CheckPointCS.SetBuffer(kernelCheckPoint, ShaderParams.TriangleBufID, triangleBuffer);
                    CheckPointCS.SetBuffer(kernelCheckPoint, ShaderParams.BvhBufID, bvhBuffer);
                    var innerPoints = GetPointInModelCS(kernelCheckPoint, newVerts, CheckPointCS, 0.5f * h);
                    tetVerts.AddRange(innerPoints);
                }
            }
            else
            {
                HashSet<float3> vertexHash = new HashSet<float3>();
                //使用哈希表剔除重复顶点,随机偏移后存入列表
                for (int i = 0; i < inputVertsCount; i++)
                {
                    if (vertexHash.Contains(vertices[i])) continue;
                    vertexHash.Add(vertices[i]);
                    tetVerts.Add(new float4(OffsetVetices(ref random, vertices[i], radius), 1));
                }
                List<float4> newVerts = new List<float4>((int)math.pow(resolution + 1, 3));
                float dim = math.max(math.max(size[0], size[1]), size[2]);
                float h = dim / resolution;
                for (int i = 0; i < (int)(size[0] / h) + 1; i++)
                {
                    float x = rootBvh.Min[0] + i * h;
                    for (int j = 0; j < (int)(size[1] / h) + 1; j++)
                    {
                        float y = rootBvh.Min[1] + j * h;
                        for (int k = 0; k < (int)(size[2] / h) + 1; k++)
                        {
                            float z = rootBvh.Min[2] + k * h;
                            // float z = rootBvh.Min[2] + k * h + RandomEps(ref random, radius);
                            var pos = new float3(x, y, z);
                            if (vertexHash.Contains(pos)) continue;
                            vertexHash.Add(pos);
                            newVerts.Add(new float4(OffsetVetices(ref random, pos, radius), 1));
                        }
                    }
                }
                //如果实际有添加有效的内部采样点，则使用ComputeShader仅保留在模型内部的采样点
                if (newVerts.Count > 0)
                {
                    int kernelCheckPoint = CheckPointCS.FindKernel(ShaderParams.KernelCheckPoint);
                    CheckPointCS.SetBuffer(kernelCheckPoint, ShaderParams.TriangleBufID, triangleBuffer);
                    CheckPointCS.SetBuffer(kernelCheckPoint, ShaderParams.BvhBufID, bvhBuffer);
                    var innerPoints = GetPointInModelCS(kernelCheckPoint, newVerts, CheckPointCS, 0.5f * h);
                    tetVerts.AddRange(innerPoints);
                }
            }

            //生成最大的外包四面体，使用半径十倍作为四面体边长,注意参数,检查顶点位置和tetTris所形成的三角面法线不能朝着四面体内部
            float s = 5f * radius;
            tetVerts.Add(new float4(center + new float3(s, 0, -s), 1));
            tetVerts.Add(new float4(center + new float3(-s, 0, -s), 1));
            tetVerts.Add(new float4(center + new float3(0, s, s), 1));
            tetVerts.Add(new float4(center + new float3(0, -s, s), 1));
            // tetVerts.Add(new float4(center + new float3(2*s, -s, 0), 1));
            // tetVerts.Add(new float4(center + new float3(-s, 2*s, 0), 1));
            // tetVerts.Add(new float4(center + new float3(-s, -s, s), 1));
            // tetVerts.Add(new float4(center + new float3(-s, -s, -s), 1));
            var tets = CreateTets(tetVerts.ToArray(), minQuality);
            //收尾：使用cs移除所有中心点在模型外部的四面体
            int kernelCheckTet = CheckPointCS.FindKernel(ShaderParams.KernelCheckTet);
            CheckPointCS.SetBuffer(kernelCheckTet, ShaderParams.TriangleBufID, triangleBuffer);
            CheckPointCS.SetBuffer(kernelCheckTet, ShaderParams.BvhBufID, bvhBuffer);
            var finalTets = GetTetInModelCS(kernelCheckTet, tetVerts, tets, CheckPointCS);
            int vertsCount = tetVerts.Count;
            tetVerts.RemoveRange(vertsCount - 4, 4);//移除最后四个大顶点
            var finalVerts = tetVerts.ToArray();
            bvhAsset.ClearComputeBuffers();
            Debug.Log($"四面体构造完成，共生成{finalTets.Length}个四面体");
            return (finalVerts, finalTets);
        }
        public static int4[] CreateTets(float4[] verts, float minQuality)
        {
            //变量声明
            //存储四面体的vertices编号，如果四面体被删除,则其对应的第一位被置为-1，第二位更新为frontDeleteTet，这样可以维护一个链表查找所有被删除的四面体
            var vertsCount = verts.Length;
            List<int> tets = new List<int>(vertsCount);
            List<int> neighbors = new List<int>(vertsCount);
            List<int> tetMarks = new List<int>(vertsCount);//洪泛算法的标记位,需要跟tets一一对应，需要同时插入
            int[] tempTet = new int[4];//用来缓存当前操作冲突四面体的四个顶点
                                       //用来缓存当前操作冲突四面体的四个邻居.因为冲突四面体空间被删除利用,在构造第一个新四面体时,其0号邻居正确,123号邻居就被覆盖为-1
                                       //这样在构造后续四面体时读取的邻居就是被覆盖的错误值,必须缓存
            int[] tempNs = new int[4];
            int tetmark = 0;
            int frontDeleteTet = -1;//记录前一个被删除四面体的编号
            List<float3> planesN = new List<float3>(vertsCount);//所有法线都朝向四面体外面为前提
            List<float> planesD = new List<float>(vertsCount);
            int firstBig = verts.Length - 4;
            //先构造大四面体
            tets.Add(firstBig);
            tets.Add(firstBig + 1);
            tets.Add(firstBig + 2);
            tets.Add(firstBig + 3);
            tetMarks.Add(0);//对应初始第一个大四面体
            for (int i = 0; i < 4; i++)
            {
                neighbors.Add(-1);
                float3 p0 = verts[firstBig + TETTRIS[i, 0]].xyz;
                float3 p1 = verts[firstBig + TETTRIS[i, 1]].xyz;
                float3 p2 = verts[firstBig + TETTRIS[i, 2]].xyz;
                float3 n = math.normalizesafe(math.cross(p1 - p0, p2 - p0));
                planesN.Add(n);
                planesD.Add(math.dot(n, p0));
            }
            //逐个插入所有顶点
            for (int index = 0; index < firstBig; index++)
            {
                int tetNum = 0;
                while (tets[4 * tetNum] < 0)//如果四面体被删除,指针后移直至找到未被删除的四面体
                {
                    tetNum++;
                }
                // //用射线相邻法找到包含顶点p的四面体
                bool found = false;
                tetmark = tetmark + 1;
                float3 pos = verts[index].xyz;

                while (!found)
                {
                    if (tetNum < 0)//边缘四面体，无邻接四面体，p在边界外
                    {
                        Debug.Log("p在边界外");
                        break;
                    }
                    else if (tetMarks[tetNum] == tetmark)//如果四面体已经被查找过，则跳过，直接使用此四面体
                    {
                        //回查代表出错,点P不在任何四面体内
                        //Debug.LogError("回查出错:" + index);
                        break;
                    }
                    tetMarks[tetNum] = tetmark;
                    int triNum = -1;
                    float tMin = float.MaxValue;
                    var p0 = verts[tets[tetNum * 4]].xyz;
                    var p1 = verts[tets[tetNum * 4 + 1]].xyz;
                    var p2 = verts[tets[tetNum * 4 + 2]].xyz;
                    var p3 = verts[tets[tetNum * 4 + 3]].xyz;
                    float3 center = 0.25f * (p0 + p1 + p2 + p3);//四面体重心
                    for (int i = 0; i < 4; i++)//判断点p是否在tetNum编号的四面体中，这个算法是难点
                    {
                        float3 n = planesN[tetNum * 4 + i];
                        float d = planesD[tetNum * 4 + i];
                        float ch = d - math.dot(n, center);//center到当前三角面法线方向投影距离
                        float ph = d - math.dot(n, pos);//p点到当前三角面法线方向投影距离
                        float t = ch - ph;//center到p点距离在当前三角面法线方向的投影,可以理解单位向量在法线方向的速度
                                          //这里最开始思考错误,以为t==0表示重合,其实这个只表示了p点到平面和center到平面距离相等,即从当前面方向永远也不可能到达P点
                        if (t < FLOAT_EPSLON) continue;
                        t = ch / t; //这里t=ch/cp,即假定c点到p点时间恒定，这里就是c点到各个三角面的时间，找出最小的t,就等于找到第一个相交的三角面
                        if (t < 0) continue; //t小于0代表p到c的矢量在此平面背面方向，只能越来越远，这里找大于0的最小值即当前四面体与cp最早交点
                        else if (t < tMin)
                        {
                            tMin = t;
                            triNum = i;
                        }
                    }
                    // if (index == 135 || index == 283)
                    // {
                    //     var circum = GetCircumCenter(verts[tets[tetNum * 4]].xyz, verts[tets[tetNum * 4 + 1]].xyz, verts[tets[tetNum * 4 + 2]].xyz, verts[tets[tetNum * 4 + 3]].xyz);
                    //     Debug.Log($"顶点{index}的容差为{tMin},重心坐标为{GetBaryCentroidCoodinate(p0,p1,p2,p3,pos)},p与外接球距离差为{math.length(pos - p0) - math.length(circum - p0)}");
                    // }
                    //即ch>cp=>cp<ch即当前center到p点距离比到任何一个面的距离都短，即p在当前四面体内
                    //!!!!!这里有一个情况,就是每次的t都等于0或者为负值,比如顶点和四面体某一顶点重合就会如此
                    //解决方式,插入顶点前对顶点去重
                    if (tMin > 1.0f - FLOAT_EPSLON || math.abs(tMin) < FLOAT_EPSLON)
                    {
                        found = true;
                    }
                    else //小于1说明p在四面体外,继续往外查
                    {
                        tetNum = neighbors[tetNum * 4 + triNum];//射线穿过哪个三角面，就选这个三角面对应的邻居重新检测，直到找到为止
                    }

                }
                if (!found)//在tetNum = -1时退出上面循环,无法使用此算法找到包含点P的四面体
                {

                    // Debug.Log("顶点" + index + "插入失败");
                    continue;
                }
                //找到包含p的四面体后，从此四面体向其邻居使用洪泛算法遍历，找到所有与p点Delaunay冲突的四面体
                tetmark++;
                List<int> violetingTets = new List<int>(vertsCount);//用来存结果
                Stack<int> checkStack = new Stack<int>();//构造堆栈，初始只存放前面找到的四面体编号
                checkStack.Push(tetNum);
                while (checkStack.Count > 0)
                {
                    tetNum = checkStack.Pop();
                    if (tetMarks[tetNum] == tetmark) continue;
                    tetMarks[tetNum] = tetmark;
                    violetingTets.Add(tetNum);
                    for (int i = 0; i < 4; i++)
                    {
                        int n = neighbors[4 * tetNum + i];
                        if (n < 0 || tetMarks[n] == tetmark) continue;
                        float3 center = GetCircumCenter(verts[tets[4 * n]].xyz, verts[tets[4 * n + 1]].xyz, verts[tets[4 * n + 2]].xyz, verts[tets[4 * n + 3]].xyz);
                        float r = math.lengthsq(center - verts[tets[4 * n]].xyz);
                        float d = math.lengthsq(center - pos);
                        if (d < r + FLOAT_EPSLON)
                        {
                            checkStack.Push(n);
                        }
                    }
                }
                var violetTettCount = violetingTets.Count;
                // if (violetingTets.Count == 0)
                // {
                //     Debug.LogError("顶点:" + index + "未找到冲突四面体");
                // }

                List<TetraEdge> edges = new List<TetraEdge>(3 * violetTettCount);

                //处理冲突四面体，将其顶点第一位标记为-1，第二位用来构造链表
                //foreach (int num in violetingTets)
                for (int t = 0; t < violetingTets.Count; t++)
                {
                    int num = violetingTets[t];
                    //移除冲突四面体前先缓存此四面体的四个顶点和邻居
                    for (int i = 0; i < 4; i++)
                    {
                        tempTet[i] = tets[4 * num + i];
                        tempNs[i] = neighbors[4 * num + i];
                    }
                    //移除和标记冲突四面体
                    tets[4 * num] = -1;
                    tets[4 * num + 1] = frontDeleteTet;
                    frontDeleteTet = num;

                    int NewTetNum;
                    //构造新四面体，访问其四个邻居，如果邻居属于冲突四面体或新生成的四面体则跳过构造，如果邻居不存在（边缘）或者不属于冲突四面体则沿着相邻面构造新四面体
                    //对于新四面体，目前仅能确定其相邻面的邻居是本次遍历的冲突四面体。其余填充-1，将本次用于构造的相邻面每条边都存入一个数组中
                    //本次顶点处理的最后根据边的相等关系来更新邻居列表
                    for (int i = 0; i < 4; i++)
                    {
                        if (tempNs[i] >= 0 && (tetMarks[tempNs[i]] == tetmark))
                        {
                            continue;
                        }
                        if (frontDeleteTet >= 0)//这个指针的作用是节约空间，如果前面已经有删除的四面体，则让出已删除四面体的地址来填充新的四面体
                        {
                            NewTetNum = frontDeleteTet;
                            frontDeleteTet = tets[4 * frontDeleteTet + 1];//用掉了旧把指针前移，直到已删除的空间用尽
                        }
                        else
                        {
                            NewTetNum = (int)math.floor(tets.Count / 4);//如果已经没有被删除四面体的剩余空间了，那就在当前列表末尾插入
                                                                        //开辟存储空间,需要更新tets neighbors planesN PlanesD tetmarks因为是List,tet列表每一次增加都要更新长度
                            for (int j = 0; j < 4; j++)
                            {
                                tets.Add(-1);
                                neighbors.Add(-1);
                                planesN.Add(float3.zero);
                                planesD.Add(0);
                            }
                            tetMarks.Add(0);
                        }
                        //新四面体的第一个邻居是本次所处理的冲突四面体当前三角面邻居（因为012对应的三角面就对应这个邻居）,其他暂时填充-1,等待后续排序填充
                        neighbors[4 * NewTetNum] = tempNs[i];
                        neighbors[4 * NewTetNum + 1] = -1;
                        neighbors[4 * NewTetNum + 2] = -1;
                        neighbors[4 * NewTetNum + 3] = -1;
                        for (int j = 0; j < 4; j++)//顺便也要改这个邻居的邻接列表，遍历它的邻接列表，如果邻居是旧四面体，改为新四面体
                        {
                            if (tempNs[i] < 0) continue;//自身在边缘邻居不存在,自然无需修改邻居的邻居标记
                            if (neighbors[tempNs[i] * 4 + j] == num)
                            {
                                neighbors[tempNs[i] * 4 + j] = NewTetNum;
                            }
                        }

                        //当前构造四面体,前三个点是所处理三角面的顶点,最后一个是插入顶点p点
                        tets[4 * NewTetNum + 0] = tempTet[TETTRIS[i, 0]];
                        tets[4 * NewTetNum + 1] = tempTet[TETTRIS[i, 1]];
                        tets[4 * NewTetNum + 2] = tempTet[TETTRIS[i, 2]];
                        tets[4 * NewTetNum + 3] = index;
                        //这里需要保证法线朝外，由于浮点误差可能求出错误的方向
                        float3 center = 0.25f * (verts[tets[4 * NewTetNum]].xyz + verts[tets[4 * NewTetNum + 1]].xyz + verts[tets[4 * NewTetNum + 2]].xyz + verts[index].xyz);
                        for (int j = 0; j < 4; j++)
                        {
                            float3 p0 = verts[tets[4 * NewTetNum + TETTRIS[j, 0]]].xyz;
                            float3 p1 = verts[tets[4 * NewTetNum + TETTRIS[j, 1]]].xyz;
                            float3 p2 = verts[tets[4 * NewTetNum + TETTRIS[j, 2]]].xyz;
                            float3 n = math.normalize(math.cross(p1 - p0, p2 - p0));
                            if (math.dot(n, center - p0) > 0) n = -n;//如果法线方向朝内则反转
                            planesN[4 * NewTetNum + j] = n;
                            planesD[4 * NewTetNum + j] = math.dot(n, p0);
                        }
                        //本次构造新四面体ABCP，将ABC的三条边按照小的顶点在前/大的顶点在后的关系存入Edge列表，本批次的新四面体如果边相同，可以推断相邻关系
                        //这里要参考顶上定义的三角面顺序{ 0, 1, 2 }, { 0, 2, 3 }, { 1, 3, 2 }, { 0, 3, 1 }
                        //(0,2)对应相邻面位置1，(1,2)对应相邻面位置2，(0,1)对应相邻面位置3
                        int d0 = tets[4 * NewTetNum + 0];
                        int d1 = tets[4 * NewTetNum + 1];
                        int d2 = tets[4 * NewTetNum + 2];
                        edges.Add((d0 < d2) ? new TetraEdge(d0, d2, NewTetNum, 1) : new TetraEdge(d2, d0, NewTetNum, 1));
                        edges.Add((d1 < d2) ? new TetraEdge(d1, d2, NewTetNum, 2) : new TetraEdge(d2, d1, NewTetNum, 2));
                        edges.Add((d0 < d1) ? new TetraEdge(d0, d1, NewTetNum, 3) : new TetraEdge(d1, d0, NewTetNum, 3));
                    }
                }
                //对于顶点P，本次的冲突四面体整理完成，接下来根据edges梳理相邻关系
                // edges.Sort(CompareEdges);
                edges.Sort((a, b) =>
                {
                    int cmp = a.p0.CompareTo(b.p0);
                    if (cmp != 0) return cmp;
                    cmp = a.p1.CompareTo(b.p1);
                    return cmp != 0 ? cmp : a.tet.CompareTo(b.tet); // 增加tet比较确保稳定
                });
                for (int i = 0; i < edges.Count - 1; i++)
                {
                    if (EqualEdges(edges[i], edges[i + 1]))//边相等则说明四面体邻接,设置互相邻接关系
                    {
                        neighbors[4 * edges[i].tet + edges[i].face] = edges[i + 1].tet;
                        neighbors[4 * edges[i + 1].tet + edges[i + 1].face] = edges[i].tet;
                        i++;//一条边最多一个邻接三角形，所以一旦判断相等就可以跳过下一个
                    }
                }
                //Debug.Log("Edge列表排序完成调试断点");
            }
            //Debug.Log("顶点插入完成");
            //全部构造完成，移除已删除四面体和低质量四面体
            int numTets = (int)math.floor(tets.Count / 4);
            int count = 0;
            int countBad = 0;
            for (int i = 0; i < numTets; i++)
            {
                int id0 = tets[4 * i];
                int id1 = tets[4 * i + 1];
                int id2 = tets[4 * i + 2];
                int id3 = tets[4 * i + 3];
                if (id0 < 0 || id0 >= firstBig || id1 >= firstBig || id2 >= firstBig || id3 >= firstBig) { continue; }//跳过删除四面体和与初始顶点相连的四面体
                float3 p0 = verts[id0].xyz;
                float3 p1 = verts[id1].xyz;
                float3 p2 = verts[id2].xyz;
                float3 p3 = verts[id3].xyz;
                float quality = TetQuality(p0, p1, p2, p3);
                if (quality < minQuality)
                {
                    countBad++;
                    continue;
                }
                //将有用的四面体重新写入原列表
                for (int j = 0; j < 4; j++)
                {
                    tets[4 * count + j] = tets[4 * i + j];
                }
                count++;//count是计数器，count后面的所有四面体最后都可以移除
            }
            // Debug.Log($"四面体构造完成，已移除{0}个外部四面体，{countBad}个低质量四面体,生成{count}个四面体,{numTets - count - countBad}个四面体被删除");
            //删除count之后的多余元素，考虑性能为了避免操作list的时候移动数组，这里从尾部开始删除，参考https://www.cnblogs.com/talentzemin/p/18409748
            int size = tets.Count;
            tets.RemoveRange(count * 4, size - count * 4);
            return Enumerable.Range(0, count).Select(i => { return new int4(tets[4 * i], tets[4 * i + 1], tets[4 * i + 2], tets[4 * i + 3]); }).ToArray();
        }
        public static float4[] GetPointInModelCS(int kernel, List<float4> newVerts, ComputeShader cs, float minDist)
        {
            //说明：最开始是使用AppendBuffer来收集结果，但是由于编辑器模式下申请的ComputerBuffer并不会清空，导致之前操作的结果污染后面的操作
            //特别是Append类型，所以改用相同长度的int类型RWStructure来收集结果，在CS中将判断在内部的设为1
            cs.GetKernelThreadGroupSizes(kernel, out var THREAD_X, out var _, out var _);
            var vertsCount = newVerts.Count;
            var resultVertsTag = new int[vertsCount];
            //指定CS中其他参数，调用CS找出newVerts中在内部的合适点
            using ComputeBuffer pointsBuffer = new ComputeBuffer(vertsCount, Marshal.SizeOf<float4>());
            pointsBuffer.SetData(newVerts);
            using ComputeBuffer pointResultBuffer = new ComputeBuffer(vertsCount, Marshal.SizeOf<int>());
            pointResultBuffer.SetData(resultVertsTag);

            cs.SetBuffer(kernel, ShaderParams.PointsBufID, pointsBuffer);
            cs.SetBuffer(kernel, ShaderParams.PointResultBufID, pointResultBuffer);
            cs.SetFloat(ShaderParams.MinDistID, minDist);
            cs.Dispatch(kernel, (int)math.ceil(vertsCount / (float)THREAD_X), 1, 1);
            pointResultBuffer.GetData(resultVertsTag);

            List<float4> result = new List<float4>(vertsCount);
            for (int i = 0; i < vertsCount; i++)
            {
                if (resultVertsTag[i] == 1)
                {
                    result.Add(newVerts[i]);
                }
            }

            //Debug.Log($"CS计算结果初始顶点{count},计算结束顶点数{result.Count}");
            pointsBuffer.Dispose();
            pointResultBuffer.Dispose();
            return result.ToArray(); ;
        }
        public static int4[] GetTetInModelCS(int kernel, List<float4> vertices, int4[] tets, ComputeShader cs)
        {
            cs.GetKernelThreadGroupSizes(kernel, out var THREAD_X, out var _, out var _);
            var tetCount = tets.Length;
            var tetsResultTag = new int[tetCount];
            ComputeBuffer verticesBuffer = new ComputeBuffer(vertices.Count, Marshal.SizeOf<float4>());
            ComputeBuffer tetsBuffer = new ComputeBuffer(tetCount, Marshal.SizeOf<int4>());
            verticesBuffer.SetData(vertices);
            tetsBuffer.SetData(tets);
            ComputeBuffer tetResultsBuffer = new ComputeBuffer(tetCount, Marshal.SizeOf<int>());
            tetResultsBuffer.SetData(tetsResultTag);

            cs.SetBuffer(kernel, ShaderParams.verticesBufID, verticesBuffer);
            cs.SetBuffer(kernel, ShaderParams.TetsBufID, tetsBuffer);
            cs.SetBuffer(kernel, ShaderParams.TetsResultBufID, tetResultsBuffer);
            cs.Dispatch(kernel, (int)math.ceil(tetCount / (float)THREAD_X), 1, 1);//注意线程组数量必须大于1，所以向上取整

            tetResultsBuffer.GetData(tetsResultTag);
            var resultTets = new List<int4>(tetCount);
            for (int i = 0; i < tetCount; i++)
            {
                if (tetsResultTag[i] == 1)
                {
                    resultTets.Add(tets[i]);
                }
            }
            //Debug.Log($"CS计算结果初始四面体数{tetCount},计算结束四面体数{result.Count}");
            verticesBuffer.Dispose();
            tetsBuffer.Dispose();
            tetResultsBuffer.Dispose();
            return resultTets.ToArray();
        }
    }
}
