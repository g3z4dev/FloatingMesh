using NUnit.Framework.Constraints;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SubmergedPartCalculator
{
    protected Transform transform;
    protected Vector3[] vertices;
    protected Vector3[] transformedVertices;
    protected int[] indices;
    protected int triangleCount;
    protected enum TriangleLocation { Above = 1, Below = -1, Intersecting = 0 };
    protected TriangleLocation[] triangleLocations;

    protected float volume;
    protected float[] volumeElements;
    protected Vector3 centroid;
    protected Vector3[] centroidElements;

    protected SimulationScene scene;

    // Pre-allocated space for calculations
    // Tetrahedra vertices 
    protected Vector3 a, b, c, o;
    // Centroid elements
    protected Vector3 C;
    // Volume elements
    protected float V;
    // Sign of volumes:
    // (Set to -1 if normals are pointing inwards)
    protected float sign = 1;

    protected const float EPSILON = 1e-5f;

    protected void GetTriangleVertices(int i, out Vector3 a, out Vector3 b, out Vector3 c)
    {
        a = vertices[indices[3 * i]];
        b = vertices[indices[3 * i + 1]];
        c = vertices[indices[3 * i + 2]];
    }
    protected void GetTriangleTransformedVertices(int i, out Vector3 a, out Vector3 b, out Vector3 c)
    {
        a = transformedVertices[indices[3 * i]];
        b = transformedVertices[indices[3 * i + 1]];
        c = transformedVertices[indices[3 * i + 2]];
    }
    protected void UpdateTriangleLocations()
    {
        for (int i = 0; i < triangleCount; i++)
        {
            GetTriangleTransformedVertices(i, out a, out b, out c);

            // Triangle is completely BELOW the fluid level
            if (scene.fluid.IsBelow(a, b, c))
            {
                triangleLocations[i] = TriangleLocation.Below;
            }
            // Triangle is completely ABOVE the fluid level
            else if (scene.fluid.IsAbove(a,b,c))
            {
                triangleLocations[i] = TriangleLocation.Above;
            }
            // Triangle is in INTERSECTION with the fluid level
            else
            {
                triangleLocations[i] = TriangleLocation.Intersecting;
            }
        }
    }
    protected void UpdateVertexPositions()
    {
        transform.TransformPoints(vertices, transformedVertices);
    }
    protected void CalculateSubmergedTriangleContribution(Vector3 a, Vector3 b, Vector3 c, Vector3 o, out float V, out Vector3 C)
    {
        // Signed volume of (abco) tetrahedra
        V = sign * Vector3.Dot(Vector3.Cross(a - o, b - o), c - o) / 6f;
        // Centroid of (abco) tetrahedra (multiplied by signed volume)
        C = (V * (a + b + c + o) / 4f);
    }

    // If local scale of the mesh is modified
    // by transform component, we have to use the
    // same scaling also for untransformed vertices,
    // to get the correct result for volume. 
    protected void ApplyLocalScale(ref Vector3 a)
    {
        a.x *= transform.localScale.x;
        a.y *= transform.localScale.y;
        a.z *= transform.localScale.z;
    }

    // Calculating volume and centroid elements for
    // the untransformed mesh
    protected void CalculateCharacteristics()
    {
        volumeElements = new float[triangleCount];
        centroidElements = new Vector3[triangleCount];

        volume = 0;
        centroid = Vector3.zero;

        o = Vector3.zero;
        for (int i = 0; i < triangleCount; i++)
        {
            GetTriangleVertices(i, out a, out b, out c);
            ApplyLocalScale(ref a);
            ApplyLocalScale(ref b);
            ApplyLocalScale(ref c);
            CalculateSubmergedTriangleContribution(a, b, c, o, out volumeElements[i], out centroidElements[i]);
            volume += volumeElements[i];
            centroid += centroidElements[i];
        }

        if(volume < 0)
        {
            Debug.LogWarning("Negative volume detected. Trying to change signs of volume elements.");
            sign = -1;
            CalculateCharacteristics();
        }
        if(volume >= EPSILON)
        {
            centroid /= volume;
        } else
        {
            Debug.LogError("Unable to calculate submerged part. Volume cannot be zero.");
        }
    }

    public SubmergedPartCalculator(Mesh mesh, Transform transform)
    {
        this.transform = transform;

        if(!mesh.isReadable)
        {
            Debug.LogError("Input mesh is not readable. Please enable the reading/writing ability on the import page.");
        }

        vertices = mesh.vertices.Clone() as Vector3[];
        transformedVertices = new Vector3[vertices.Length];
        transform.TransformPoints(vertices, transformedVertices);
        indices = mesh.triangles.Clone() as int[];
        triangleCount = indices.Length / 3;

        triangleLocations = new TriangleLocation[triangleCount];

        CalculateCharacteristics();
    }

    public float Volume { get { return volume; } }
    public Vector3 Centroid { get { return centroid; } }

    public SimulationScene SimulationScene
    {
        get { return scene; }
        set { scene = value; }
    }
    public virtual void Calculate(out Vector3 submergedCentroid, out Vector3 buoyantForce)
    {
        UpdateVertexPositions();
        UpdateTriangleLocations();
        submergedCentroid = Vector3.zero;
        buoyantForce = Vector3.zero;
    }
}

public class VolumeBasedSPC : SubmergedPartCalculator
{
    public VolumeBasedSPC(Mesh mesh, Transform transform) : base(mesh, transform)
    { }

    // Calculate buoyant force
    // F_B = rho * V_B * g * (0,1,0)
    protected Vector3 CalculateForce(float submergedVolume)
    {
        return Vector3.up * scene.fluid.fluidDensity * submergedVolume * scene.world.gravity;
    }
}

public class FastSPC : VolumeBasedSPC
{
    public FastSPC(Mesh mesh, Transform transform) : base(mesh, transform)
    { }

    public override void Calculate(out Vector3 submergedCentroid, out Vector3 buoyantForce)
    {
        base.Calculate(out submergedCentroid, out buoyantForce);

        float submergedVolume = 0;

        // Calculate the contributions of the totally submerged triangles
        // in the local coordinate system. Volume and centroid elements 
        // are precalculated, therefore this algorithm is faster than other 2.
        for (int i = 0; i < triangleCount; i++)
        {
            if (triangleLocations[i] == TriangleLocation.Below)
            {
                submergedVolume += volumeElements[i];
                submergedCentroid += centroidElements[i];
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
    }
}

public class ImprovedSPC : VolumeBasedSPC
{
    Vector3 transformedSubmergedCentroid;
    public ImprovedSPC(Mesh mesh, Transform transform) : base(mesh, transform)
    { }

    public override void Calculate(out Vector3 submergedCentroid, out Vector3 buoyantForce)
    {
        base.Calculate(out transformedSubmergedCentroid, out buoyantForce);

        float submergedVolume = 0;

        // Transform (local) centroid to world-space
        o = transform.TransformPoint(centroid);
        // Project to the y = fluidLevel plane
        o.y = scene.fluid.GetFluidLevelAt(o.x, o.z);
        // Now o is ON the surface of the fluid!
        // This causes the volume elements to be zero for
        // each triangles ON the fluid surface.
        for (int i = 0; i < triangleCount; i++)
        {
            if (triangleLocations[i] == TriangleLocation.Below)
            {
                GetTriangleTransformedVertices(i, out a, out b, out c);
                CalculateSubmergedTriangleContribution(a, b, c, o, out V, out C);
                submergedVolume += V;
                transformedSubmergedCentroid += C;
            }
        }
        if (submergedVolume >= EPSILON)
            transformedSubmergedCentroid /= submergedVolume;
        else
            submergedVolume = 0;

        // Transform back the centroid into the local coordinate system
        submergedCentroid = transform.InverseTransformPoint(transformedSubmergedCentroid);
        buoyantForce = CalculateForce(submergedVolume);
    }
}

public class ExactSPC : VolumeBasedSPC
{
    Vector3 transformedSubmergedCentroid;
    // Intersection points
    Vector3 p, q;
    // Index containers
    int i1, i2, i3, k;
    // Submerged volume parts
    float V1, V2;
    // Submerged centroid parts
    Vector3 C1, C2;
    public ExactSPC(Mesh mesh, Transform transform) : base(mesh, transform)
    { }

    private void CalculatePartiallySubmergedTriangleContribution(int i, out float V, out Vector3 C)
    {
        V = 0;
        C = Vector3.zero;

        // Find the above-to-below edge of the triangle (it is unique).
        // The k-th vertex is ABOVE the surface,
        // the (k+1)-th vertex is BELOW the surface
        int k = -1;
        for (int j = 0; j < 3; j++)
        {
            i1 = indices[3 * i + j];
            i2 = indices[3 * i + ((j + 1) % 3)];
            if (!scene.fluid.IsBelow(transformedVertices[i1]) &&
                scene.fluid.IsBelow(transformedVertices[i2]))
            {
                k = j;
                break;
            }
        }

        if (k < 0)
        {
            Debug.LogWarning("Skipping partially submerged triangle's contribution.");
            return;
        }
        // Cyclic permutation of vertices such that
        // a->b will be the above-to-below edge.
        i1 = indices[3 * i + k];
        i2 = indices[3 * i + ((k + 1) % 3)];
        i3 = indices[3 * i + ((k + 2) % 3)];

        a = transformedVertices[i1];
        b = transformedVertices[i2];
        c = transformedVertices[i3];


        // Triangle is facing downwards:
        //
        //  a       c
        // __ p _ q __________ fluidLevel
        //      b   
        if (c.y >= 0)
        {
            scene.fluid.FluidSegmentIntersection(a, b, out p);
            scene.fluid.FluidSegmentIntersection(b, c, out q);
            CalculateSubmergedTriangleContribution(p, b, q, o, out V, out C);
        }
        // Triangle is facing upwards:
        //
        //      a
        // __ p _ q __________ fluidLevel
        //  b       c
        else
        {
            scene.fluid.FluidSegmentIntersection(a, b, out p);
            scene.fluid.FluidSegmentIntersection(c, a, out q);
            // Break quadrilateral into two triangles
            CalculateSubmergedTriangleContribution(p, b, q, o, out V1, out C1);
            CalculateSubmergedTriangleContribution(q, b, c, o, out V2, out C2);
            V = V1 + V2;
            C = C1 + C2;
        }
    }

    public override void Calculate(out Vector3 submergedCentroid, out Vector3 buoyantForce)
    {
        base.Calculate(out transformedSubmergedCentroid, out buoyantForce);
        float submergedVolume = 0;

        // The same trick as in improved algorithm.
        o = transform.TransformPoint(centroid);
        o.y = scene.fluid.GetFluidLevelAt(o.x, o.z);

        for (int i = 0; i < triangleCount; i++)
        {
            if (triangleLocations[i] == TriangleLocation.Below)
            {
                GetTriangleTransformedVertices(i, out a, out b, out c);
                CalculateSubmergedTriangleContribution(a, b, c, o, out V, out C);
                submergedVolume += V;
                transformedSubmergedCentroid += C;
            }
            // But we also calculate the contribution of the
            // partially submerged triangles
            if (triangleLocations[i] == TriangleLocation.Intersecting)
            {
                CalculatePartiallySubmergedTriangleContribution(i, out V, out C);
                submergedVolume += V;
                transformedSubmergedCentroid += C;
            }
        }
        if (submergedVolume >= EPSILON)
            transformedSubmergedCentroid /= submergedVolume;
        else
            submergedVolume = 0;

        // Transform back the centroid into the local coordinate system
        submergedCentroid = transform.InverseTransformPoint(transformedSubmergedCentroid);
        buoyantForce = CalculateForce(submergedVolume);
    }
}
