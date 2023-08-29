using System;
using UnityEngine;

public class pointCloud : MonoBehaviour
{
    public String pathToRawData;
    public String ID;

    public double adjustmentX;
    public double adjustmentY;
    public double adjustmentZ;

    public double initialXShift;
    public double initialZShift;

    public string spatialInfo;
    public int UTMZone;
    public bool North;

    public double AABB_min_x;
    public double AABB_min_y;
    public double AABB_min_z;

    public double AABB_max_x;
    public double AABB_max_y;
    public double AABB_max_z;

    public int EPSG;

    public float groundLevel;
}
