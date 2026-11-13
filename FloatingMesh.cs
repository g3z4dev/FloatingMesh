using System;
using Unity.VectorGraphics;
using Unity.VisualScripting;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(Rigidbody))]
public partial class FloatingMesh : MonoBehaviour
{
    public enum CalculationProcess { FastVolume, ImprovedVolume, ExactVolume, FastSurface, ExactSurface };
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
            case CalculationProcess.ExactSurface:
                calculator = new ExactSurfaceBFC(mesh, transform, scene);
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
        if(calculator is VolumeBasedBFC c1)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawSphere(c1.SubmergedCentroid, 0.3f);
            List<Vector3> centroids = ((SubmergedVolumeMesh)c1.MeshData).TetrahedronCentroids;
            Gizmos.color = Color.aquamarine;
            for (int i = 0; i < centroids.Count; i++)
            {
                Gizmos.DrawSphere(centroids[i], 0.1f);
            }
        } 
        if (calculator is ExactSurfaceBFC c2)
        {
            for (int i = 0; i < c2.centroids.Count; i++)
            {
                Gizmos.color = Color.pink;
                Gizmos.DrawSphere(c2.centroids[i], 0.05f);
                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(c2.centroids[i], c2.centroids[i] + c2.forces[i]);
                Gizmos.DrawCube(c2.centroids[i] + c2.forces[i], new Vector3(0.05f, 0.05f, 0.05f));
                Gizmos.color = Color.red;
                Gizmos.DrawLine(c2.centroids[i], c2.centroids[i] + c2.torques[i]);
                Gizmos.DrawCube(c2.centroids[i] + c2.torques[i], new Vector3(0.05f, 0.05f, 0.05f));
            }
            for (int i = 0; i < c2.intersections.Count; i += 3)
            {
                Gizmos.color = Color.limeGreen;
                Gizmos.DrawLine(c2.intersections[i], c2.intersections[i+1]);
                Gizmos.DrawLine(c2.intersections[i+1], c2.intersections[i + 2]);
                Gizmos.DrawLine(c2.intersections[i+2], c2.intersections[i]);
            }
        }
        if (calculator is SurfaceBasedBFC c3)
        {
            List<Vector3> normals = ((SubmergedAreaMesh)c3.MeshData).Normals;
            List<Vector3> centroids = ((SubmergedAreaMesh)c3.MeshData).Centroids;
            for (int i = 0; i < centroids.Count; i++)
            {
                Gizmos.color = Color.aquamarine;
                Gizmos.DrawSphere(centroids[i], 0.05f);
                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(centroids[i], centroids[i] + normals[i]);
                Gizmos.DrawCube(centroids[i] + normals[i], new Vector3(0.05f, 0.05f, 0.05f));
            }
        }
    }
}