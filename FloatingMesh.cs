using System;
using Unity.VectorGraphics;
using Unity.VisualScripting;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(Rigidbody))]
public partial class FloatingMesh : MonoBehaviour
{
    public enum CalculationProcess { FastVolume, ImprovedVolume, ExactVolume, FastSurface };
    public CalculationProcess calculationProcess = CalculationProcess.FastVolume;

    Mesh mesh;
    
    Rigidbody rigidbody;

    [Header("Fluid Settings")]
    public float fluidLevel = 0f;
    [SerializeField]
    public float fluidDensity = 1f;
    [Header("Floating Body")]

    private SimulationScene scene = new();

    Vector3 buoyantTorque;
    Vector3 buoyantForce;

    BuoyantForceCalculator calculator;
    protected const float EPSILON = 1e-5f;

    public void Refresh()
    {
        // In each FixedUpdate cycle, submerged volume and
        // submerged centroid are recalculated.
        //
        // This calculation process can be called outside of this class,
        // which is useful when testing.
        calculator.Calculate(out buoyantTorque, out buoyantForce);
    }

    void Start()
    {
        rigidbody = GetComponent<Rigidbody>();
        Mesh mesh = GetComponent<MeshFilter>().sharedMesh;

        // Set strategy for submerged volume calculation
        switch (calculationProcess)
        {
            case CalculationProcess.FastVolume:
                calculator = new FastVolumeBFC(mesh, transform, scene);
                break;
            case CalculationProcess.ImprovedVolume:
                calculator = new ImprovedVolumeBFC(mesh, transform, scene);
                break;
            case CalculationProcess.ExactVolume:
                calculator = new ExactVolumeBFC(mesh, transform, scene);
                break;
            case CalculationProcess.FastSurface:
                calculator = new FastSurfaceBFC(mesh, transform, scene);
                break;
        }

        rigidbody.centerOfMass = calculator.LocalCentroid;
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
        rigidbody.AddForce(buoyantForce);
        rigidbody.AddTorque(buoyantTorque);
    }

    private void FixedUpdate()
    {
        if (rigidbody.isKinematic)
            return;

        // Calculate submerged volume and submerged centroid
        Refresh();

        AddBuoyantForce();
    }

    private void OnDrawGizmos()
    {
        if (calculator == null) return;
        Gizmos.color = Color.orange;
        Gizmos.DrawSphere(calculator.MeshData.Centroid, 0.3f);
        Gizmos.color = Color.green;
        Vector3 fG = Vector3.down * rigidbody.mass * scene.world.gravity;
        Gizmos.DrawLine(calculator.MeshData.Centroid, calculator.MeshData.Centroid + buoyantForce + fG);
        Gizmos.DrawCube(calculator.MeshData.Centroid + buoyantForce + fG, new Vector3(0.2f, 0.2f, 0.2f));
        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(calculator.MeshData.Centroid, calculator.MeshData.Centroid + buoyantForce);
        Gizmos.DrawCube(calculator.MeshData.Centroid + buoyantForce, new Vector3(0.2f, 0.2f, 0.2f));
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(calculator.MeshData.Centroid, calculator.MeshData.Centroid + fG);
        Gizmos.DrawCube(calculator.MeshData.Centroid + fG, new Vector3(0.2f, 0.2f, 0.2f));
        Gizmos.color = Color.red;
        Gizmos.DrawLine(calculator.MeshData.Centroid, calculator.MeshData.Centroid + buoyantTorque);
        Gizmos.DrawCube(calculator.MeshData.Centroid + buoyantTorque, new Vector3(0.2f, 0.2f, 0.2f));
    }
}