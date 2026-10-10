using System;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(Rigidbody))]
public partial class FloatingMesh : MonoBehaviour
{
    public enum CalculationProcess { Fast, Improved, Exact };
    public CalculationProcess calculationProcess = CalculationProcess.Fast;

    Mesh mesh;
    
    Rigidbody rigidbody;

    [Header("Fluid Settings")]
    public float fluidLevel = 0f;
    [SerializeField]
    public float fluidDensity = 1f;
    [Header("Floating Body")]

    private SimulationScene scene = new();

    Vector3 submergedCentroid;
    Vector3 buoyantForce;
    Vector3 transformedSubmergedCentroid;

    SubmergedPartCalculator calculator;
    Vector3 centroid;
    protected const float EPSILON = 1e-5f;

    public void Refresh()
    {
        // In each FixedUpdate cycle, submerged volume and
        // submerged centroid are recalculated.
        //
        // This calculation process can be called outside of this class,
        // which is useful when testing.
        calculator.Calculate(out submergedCentroid, out buoyantForce);
    }

    void Start()
    {
        rigidbody = GetComponent<Rigidbody>();
        Mesh mesh = GetComponent<MeshFilter>().sharedMesh;

        // Set strategy for submerged volume calculation
        switch (calculationProcess)
        {
            case CalculationProcess.Fast:
                calculator = new FastSPC(mesh, transform);
                break;
            case CalculationProcess.Improved:
                calculator = new ImprovedSPC(mesh, transform);
                break;
            case CalculationProcess.Exact:
                calculator = new ExactSPC(mesh, transform);
                break;
        }

        calculator.SimulationScene = scene;
        rigidbody.centerOfMass = calculator.Centroid;
        this.scene.fluid.fluidLevel = fluidLevel;
        this.scene.fluid.fluidDensity = fluidDensity;
        this.scene.floatingObject.mass = rigidbody.mass;
        this.scene.world.gravity = Physics.gravity.magnitude;
    }

    private void AddBuoyantForce()
    {
        if (buoyantForce.magnitude < EPSILON)
            return;

        // Add buoyant force with built-in rigidbody method.
        // The reference point of this force is the transformed submerged centroid.
        rigidbody.AddForceAtPosition(buoyantForce, transformedSubmergedCentroid, ForceMode.Force);
    }

    private void FixedUpdate()
    {
        if (rigidbody.isKinematic)
            return;

        // Calculate submerged volume and submerged centroid
        Refresh();

        // Transform submerged centroid (from local coordinate system)
        // to world space
        transformedSubmergedCentroid = transform.TransformPoint(submergedCentroid);

        AddBuoyantForce();
    }
}

public class Fluid
{
    public float fluidLevel;
    public float fluidDensity;
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
}

public class FloatingObject
{
    public float mass;
}

public class World
{
    public float gravity;
}

public class SimulationScene
{
    public Fluid fluid = new();
    public FloatingObject floatingObject = new();
    public World world = new();
}