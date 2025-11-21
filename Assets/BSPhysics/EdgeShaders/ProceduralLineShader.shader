Shader "Hidden/LineShader" 
{
    SubShader 
    {
        Pass 
        {
            
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5
            #include "UnityShaderVariables.cginc"
            struct Edge
            {
                int2 index;
                float length;
            };
            StructuredBuffer<float4> _Positions;
            StructuredBuffer<Edge> _Edges;
            StructuredBuffer<float4x4> _LocalToWorldMatrixs;
            int edgeCount;
            int tetVtxCount;
            fixed4 _Color;
            struct v2f 
            {
                float4 pos : SV_POSITION;
            };
            v2f vert(uint vertexID : SV_VertexID, uint instanceID : SV_InstanceID) 
            {
                v2f o;
                // 获取当前边的两个顶点索引
                int edgeIndex = instanceID % edgeCount;
                int objectIndex = instanceID/edgeCount;
                int2 edge = _Edges[edgeIndex].index;
                //这里要加上偏移值
                float4 p1 = _Positions[tetVtxCount*objectIndex+edge.x];
                float4 p2 = _Positions[tetVtxCount*objectIndex+edge.y];
                float4x4 mat = _LocalToWorldMatrixs[objectIndex];
                // 根据vertexID选择起点或终点
                float4 objectPos = (vertexID == 0) ? p1 : p2;
                float4 wPos =mul(mat,objectPos);
                o.pos = mul(UNITY_MATRIX_VP,wPos);
                // o.pos = objectPos;
                // o.pos = UnityObjectToClipPos(wPos);
                return o;
            }
            fixed4 frag(v2f i) : SV_Target 
            {
                return _Color; // 黄色线条
            }
            ENDCG
        }
    }
}