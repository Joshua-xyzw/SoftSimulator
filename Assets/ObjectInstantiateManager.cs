using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using Random = UnityEngine.Random;
using BSPhysics;
using UnityEngine.UI;

public class ObjectInstantiateManager : MonoBehaviour
{
    public GameObject SimulatePrefab;
    public void Awake()
    {
        
    }
    public void CreateObject()
    {
        float3 pos = new float3(-15f + 30f * Random.value, 5, Random.value * 40);
        quaternion quaternion = Quaternion.Euler(0, 360f * Random.value, 0);
        var soft = GameObject.Instantiate(SimulatePrefab, pos, quaternion);
    }
}
