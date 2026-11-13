using NUnit.Framework.Constraints;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public abstract class BuoyantForceCalculator
{
    protected SubmergedMesh mesh;
    protected SimulationScene scene;
    protected Vector3 a, b, c;

    protected const float EPSILON = 1e-5f;

    public BuoyantForceCalculator(Mesh mesh, Transform transform, SimulationScene scene)
    {
        this.scene = scene;
        if(!mesh.isReadable)
        {
            Debug.LogError("Input mesh is not readable. Please enable the reading/writing ability on the import page.");
        }
    }
    public virtual void Calculate(out Vector3 buoyantTorque, out Vector3 buoyantForce)
    {
        mesh.Update();
        buoyantTorque = Vector3.zero;
        buoyantForce = Vector3.zero;
    }
    public Vector3 CalculateTorque(Vector3 force, Vector3 point, Vector3 com)
    {
        return Vector3.Cross(point - com, force);
    }

    public Vector3 Centroid
    {
        get => mesh.Centroid;
    }
    public Vector3 LocalCentroid
    {
        get => mesh.LocalCentroid;
    }

    public SubmergedMesh MeshData
    {
        get => mesh;
    }
}

public abstract class VolumeBasedBFC : BuoyantForceCalculator
{
    protected Vector3 submergedCentroid;
    public Vector3 SubmergedCentroid
    {
        get => submergedCentroid;
    }
    protected SubmergedVolumeMesh Mesh
    {
        get => (SubmergedVolumeMesh) mesh;
    }
    public float Volume { get { return Mesh.Volume; } }
    public VolumeBasedBFC(Mesh mesh, Transform transform, SimulationScene scene) : base(mesh, transform, scene)
    {
        this.mesh = new SubmergedVolumeMesh(mesh, transform, scene);
    }

    // Calculate buoyant force
    // F_B = rho * V_B * g * (0,1,0)
    protected Vector3 CalculateForce(float submergedVolume)
    {
        return Vector3.up * scene.fluid.fluidDensity * submergedVolume * scene.world.gravity;
    }
}

public class SurfaceBasedBFC : BuoyantForceCalculator
{
    protected SubmergedAreaMesh Mesh
    {
        get => (SubmergedAreaMesh)mesh;
    }
    public SurfaceBasedBFC(Mesh mesh, Transform transform, SimulationScene scene) : base(mesh, transform, scene)
    {
        this.mesh = new SubmergedAreaMesh(mesh, transform, scene);
    }

    // Calculate buoyant force contribution of a discrete wet surface
    // F_i = p(c) * A_i * \hat{n}
    // This is the discretization of the integral:
    // F = -\int\int_(S)p\hat{n}dS
    protected Vector3 CalculateForce(Vector3 normal, Vector3 centroid)
    {
        return -scene.fluid.GetHydrostaticPressure(centroid) * normal;
    }
}

public class FastVolumeBFC : VolumeBasedBFC
{
    public FastVolumeBFC(Mesh mesh, Transform transform, SimulationScene scene) : base(mesh, transform, scene)
    { }

    public override void Calculate(out Vector3 buoyantTorque, out Vector3 buoyantForce)
    {
        base.Calculate(out buoyantTorque, out buoyantForce);

        float submergedVolume = 0;
        submergedCentroid = Vector3.zero;

        // Calculate the contributions of the totally submerged triangles
        // in the local coordinate system. Volume and centroid elements 
        // are precalculated, therefore this algorithm is faster than other 2.
        for (int i = 0; i < Mesh.TriangleCount; i++)
        {
            if (Mesh.IsBelow(i))
            {
                submergedVolume += Mesh.GetVolumeContributionOfTriangle(i);
                submergedCentroid += Mesh.GetVolumeContributionOfTriangle(i)*Mesh.GetTetrahedronCentroidOfTriangle(i);
            }
        }
        if (submergedVolume >= EPSILON)
        {
            submergedCentroid /= submergedVolume;
        }
        else
        {
            submergedVolume = 0;
        }

        buoyantForce = CalculateForce(submergedVolume);
        buoyantTorque = CalculateTorque(buoyantForce, submergedCentroid, Mesh.Centroid);
    }
}

public class ImprovedVolumeBFC : VolumeBasedBFC
{
    private float V;
    private Vector3 C, o;
    public ImprovedVolumeBFC(Mesh mesh, Transform transform, SimulationScene scene) : base(mesh, transform, scene)
    { }

    public override void Calculate(out Vector3 buoyantTorque, out Vector3 buoyantForce)
    {
        base.Calculate(out buoyantTorque, out buoyantForce);

        submergedCentroid = Vector3.zero;
        float submergedVolume = 0;

        // Transform (local) centroid to world-space
        o = Mesh.Centroid;
        // Project to the y = fluidLevel plane
        o.y = scene.fluid.GetFluidLevelAt(o.x, o.z);
        // Now o is ON the surface of the fluid!
        // This causes the volume elements to be zero for
        // each triangles ON the fluid surface.
        for (int i = 0; i < Mesh.TriangleCount; i++)
        {
            if (Mesh.IsBelow(i))
            {
                Mesh.GetTriangleVertices(i, out a, out b, out c);
                Mesh.CalculateSubmergedTriangleContribution(a, b, c, o, out V, out C);
                submergedVolume += V;
                submergedCentroid += V*C;
            }
        }
        if (submergedVolume >= EPSILON)
            submergedCentroid /= submergedVolume;
        else
            submergedVolume = 0;

        // Transform back the centroid into the local coordinate system;
        buoyantForce = CalculateForce(submergedVolume);
        buoyantTorque = CalculateTorque(buoyantForce, submergedCentroid, Mesh.Centroid);
    }
}

public class ExactVolumeBFC : VolumeBasedBFC
{
    protected Vector3 o;
    protected float V, V1, V2;
    protected Vector3 C, C1, C2;
    protected Vector3 p, q;
    protected SubmergedMesh.TriangleOrientation orientation;
    public ExactVolumeBFC(Mesh mesh, Transform transform, SimulationScene scene) : base(mesh, transform, scene)
    { }

    public override void Calculate(out Vector3 buoyantTorque, out Vector3 buoyantForce)
    {
        base.Calculate(out buoyantTorque, out buoyantForce);

        submergedCentroid = Vector3.zero;
        float submergedVolume = 0;

        // The same trick as in improved algorithm.
        o = Mesh.Centroid;
        o.y = scene.fluid.GetFluidLevelAt(o.x, o.z);

        for (int i = 0; i < Mesh.TriangleCount; i++)
        {
            if (Mesh.IsBelow(i))
            {
                Mesh.GetTriangleVertices(i, out a, out b, out c);
                Mesh.CalculateSubmergedTriangleContribution(a, b, c, o, out V, out C);
                submergedVolume += V;
                submergedCentroid += V*C;
            }
            // But we also calculate the contribution of the
            // partially submerged triangles
            if (Mesh.IsIntersecting(i))
            {
                Mesh.GetTriangleVerticesWithIntersectingPoints(i, out a, out b, out c, out p, out q, out orientation);

                if(orientation == SubmergedMesh.TriangleOrientation.Downwards)
                {
                    Mesh.CalculateSubmergedTriangleContribution(p, b, q, o, out V, out C);
                    C *= V;
                } 
                else
                {
                    Mesh.CalculateSubmergedTriangleContribution(p, b, q, o, out V1, out C1);
                    Mesh.CalculateSubmergedTriangleContribution(q, b, c, o, out V2, out C2);
                    V = V1 + V2;
                    C = V1*C1 + V2*C2;
                }
                submergedVolume += V;
                submergedCentroid += C;
            }
        }
        if (submergedVolume >= EPSILON)
            submergedCentroid /= submergedVolume;
        else
            submergedVolume = 0;

        // Transform back the centroid into the local coordinate system
        buoyantForce = CalculateForce(submergedVolume);
        buoyantTorque = CalculateTorque(buoyantForce, submergedCentroid, Mesh.Centroid);
    }
}


public class FastSurfaceBFC : SurfaceBasedBFC
{
    protected Vector3 triangleCentroid, force;
    public FastSurfaceBFC(Mesh mesh, Transform transform, SimulationScene scene) : base(mesh, transform, scene)
    { }

    public override void Calculate(out Vector3 buoyantTorque, out Vector3 buoyantForce)
    {
        base.Calculate(out buoyantTorque, out buoyantForce);

        for (int i = 0; i < Mesh.TriangleCount; i++)
        {
            if (Mesh.IsBelow(i))
            {
                triangleCentroid = Mesh.GetCentroid(i);
                force = CalculateForce(Mesh.GetNormal(i), triangleCentroid);
                buoyantForce += force;
                buoyantTorque += CalculateTorque(force, triangleCentroid, Mesh.Centroid);
            }
        }
    }
}


public class ExactSurfaceBFC : SurfaceBasedBFC
{
    protected Vector3 triangleCentroid, force;
    protected Vector3 p, q;
    protected Vector3 normal;
    protected SubmergedMesh.TriangleOrientation orientation;
    public List<Vector3> normals;
    public List<Vector3> centroids;
    public List<Vector3> forces;
    public List<Vector3> torques;
    public List<Vector3> intersections;
    public ExactSurfaceBFC(Mesh mesh, Transform transform, SimulationScene scene) : base(mesh, transform, scene)
    { }

    public override void Calculate(out Vector3 buoyantTorque, out Vector3 buoyantForce)
    {
        base.Calculate(out buoyantTorque, out buoyantForce); 
        List<Vector3> normals = new();
        List<Vector3> centroids = new();
        List<Vector3> forces = new();
        List<Vector3> torques = new();
        List<Vector3> intersections = new();

        for (int i = 0; i < Mesh.TriangleCount; i++)
        {
            if (Mesh.IsBelow(i))
            {
                triangleCentroid = Mesh.GetCentroid(i);
                force = CalculateForce(Mesh.GetNormal(i), triangleCentroid);
                buoyantForce += force;
                buoyantTorque += CalculateTorque(force, triangleCentroid, Mesh.Centroid);

                normals.Add(Mesh.GetNormal(i));
                centroids.Add(triangleCentroid);
                forces.Add(force);
                torques.Add(CalculateTorque(force, triangleCentroid, Mesh.Centroid));
            } 
            else if (Mesh.IsIntersecting(i))
            {
                Mesh.GetTriangleVerticesWithIntersectingPoints(i, out a, out b, out c, out p, out q, out orientation);
                if (orientation == SubmergedMesh.TriangleOrientation.Downwards)
                {
                    intersections.Add(p);
                    intersections.Add(b);
                    intersections.Add(q);
                    // the normals must be calculated so that they point outwards!
                    normal = Vector3.Cross(b - p, q - p) / 2;
                    triangleCentroid = (p + b + q) / 3;
                    force = CalculateForce(normal, triangleCentroid);
                    buoyantForce += force;
                    buoyantTorque += CalculateTorque(force, triangleCentroid, Mesh.Centroid);

                    normals.Add(normal);
                    centroids.Add(triangleCentroid);
                    forces.Add(force);
                    torques.Add(CalculateTorque(force, triangleCentroid, Mesh.Centroid));
                }
                else
                {
                    intersections.Add(p);
                    intersections.Add(b);
                    intersections.Add(q);
                    intersections.Add(q);
                    intersections.Add(b);
                    intersections.Add(c);
                    // the normals must be calculated so that they point outwards!
                    normal = Vector3.Cross(b - p, q - p) / 2;
                    triangleCentroid = (p + b + q) / 3;
                    force = CalculateForce(normal, triangleCentroid);
                    buoyantForce += force;
                    buoyantTorque += CalculateTorque(force, triangleCentroid, Mesh.Centroid);

                    normals.Add(normal);
                    centroids.Add(triangleCentroid);
                    forces.Add(force);
                    torques.Add(CalculateTorque(force, triangleCentroid, Mesh.Centroid));


                    normal = Vector3.Cross(b - q, c - q) / 2;
                    triangleCentroid = (q + b + c) / 3;
                    force = CalculateForce(normal, triangleCentroid);
                    buoyantForce += force;
                    buoyantTorque += CalculateTorque(force, triangleCentroid, Mesh.Centroid);

                    normals.Add(normal);
                    centroids.Add(triangleCentroid);
                    forces.Add(force);
                    torques.Add(CalculateTorque(force, triangleCentroid, Mesh.Centroid));
                }

            }
        }

        this.normals = normals;
        this.centroids = centroids;
        this.forces = forces;
        this.torques = torques;
        this.intersections = intersections;
    }
}