using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

public class SimulationScene
{
    public World world;
    public Fluid fluid;
    public FloatingObject floatingObject;

    public SimulationScene()
    {
        world = new();
        fluid = new(world);
        floatingObject = new();
    }
}

public class Fluid
{
    private World world;
    public float fluidLevel;
    public float fluidDensity;

    public Fluid(World world)
    {
        this.world = world;
    }

    public bool IsBelow(Vector3 point)
    {
        return fluidLevel > point.y;
    }

    public bool IsBelow(Vector3 a, Vector3 b, Vector3 c)
    {
        return IsBelow(a) && IsBelow(b) && IsBelow(c);
    }

    public bool IsAbove(Vector3 point)
    {
        return fluidLevel < point.y;
    }
    public bool IsAbove(Vector3 a, Vector3 b, Vector3 c)
    {
        return IsAbove(a) && IsAbove(b) && IsAbove(c);
    }
    public bool IsIntersecting(Vector3 a, Vector3 b, Vector3 c)
    {
        return !IsBelow(a, b, c) && !IsAbove(a, b, c);
    }

    public float GetFluidLevelAt(float x, float z)
    {
        return fluidLevel;
    }

    public void FluidSegmentIntersection(Vector3 a, Vector3 b, out Vector3 x)
    {
        x = a + (b - a) * (fluidLevel - a.y) / (b.y - a.y);
    }

    public float GetHydrostaticPressure(Vector3 pos)
    {
        return fluidDensity * world.gravity * (fluidLevel - pos.y);
    }
}

public class FloatingObject
{
    public float mass;
}

public class World
{
    public float gravity;
}
