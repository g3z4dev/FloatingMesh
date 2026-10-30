using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.VectorGraphics;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Rendering;

public abstract class SubmergedMesh
{
    protected Transform transform;
    protected SimulationScene scene;
    protected Vector3[] untransformedVertices;
    protected Vector3[] transformedVertices;
    protected int[] indices;
    protected int triangleCount;
    protected Vector3 untransformedCentroid;
    protected Vector3 transformedCentroid;
    protected enum TriangleLocation { Above = 1, Below = -1, Intersecting = 0 };
    protected TriangleLocation[] triangleLocations;
    protected const float EPSILON = 1e-5f;
    protected Vector3 a, b, c;
    protected abstract void CalculateCharacteristics();

    protected SubmergedMesh(Mesh mesh, Transform transform, SimulationScene scene)
    {
        this.transform = transform;
        this.scene = scene;

        if (!mesh.isReadable)
        {
            Debug.LogError("Input mesh is not readable. Please enable the reading/writing ability on the import page.");
        }

        untransformedVertices = mesh.vertices.Clone() as Vector3[];
        transformedVertices = new Vector3[untransformedVertices.Length];
        transform.TransformPoints(untransformedVertices, transformedVertices);
        indices = mesh.triangles.Clone() as int[];
        triangleCount = indices.Length / 3;

        triangleLocations = new TriangleLocation[triangleCount];

        CalculateCharacteristics();
    }
    protected virtual void UpdateVertexPositions()
    {
        transform.TransformPoints(untransformedVertices, transformedVertices);
        transformedCentroid = transform.TransformPoint(untransformedCentroid);
    }
    protected void UpdateTriangleLocations()
    {
        for (int i = 0; i < triangleCount; i++)
        {
            GetTriangleVertices(i, out a, out b, out c);

            // Triangle is completely BELOW the fluid level
            if (scene.fluid.IsBelow(a, b, c))
            {
                triangleLocations[i] = TriangleLocation.Below;
            }
            // Triangle is completely ABOVE the fluid level
            else if (scene.fluid.IsAbove(a, b, c))
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

    public void Update()
    {
        UpdateVertexPositions();
        UpdateTriangleLocations();
    }
    public void GetTriangleVertices(int i, out Vector3 a, out Vector3 b, out Vector3 c)
    {
        a = transformedVertices[indices[3 * i]];
        b = transformedVertices[indices[3 * i + 1]];
        c = transformedVertices[indices[3 * i + 2]];
    }

    public bool IsBelow(int triangleIndex)
    {
        return triangleLocations[triangleIndex] == TriangleLocation.Below;
    }
    public bool IsIntersecting(int triangleIndex)
    {
        return triangleLocations[triangleIndex] == TriangleLocation.Intersecting;
    }

    public int TriangleCount
    {
        get => triangleCount;
    }

    public Vector3 Centroid
    {
        get => transformedCentroid;
    }
    public Vector3 LocalCentroid
    {
        get => untransformedCentroid;
    }
}

public class SubmergedVolumeMesh : SubmergedMesh
{
    protected float[] tetrahedronVolumeElements;
    protected Vector3[] untransformedTetrahedronCentroidElements;
    protected Vector3[] transformedTetrahedronCentroidElements;
    protected float volume;
    protected float sign = 1;
    protected Vector3 o;
    // Intersection points
    Vector3 p, q;
    // Index containers
    int i1, i2, i3, k;
    // Submerged volume parts
    float V1, V2;
    // Submerged centroid parts
    Vector3 C1, C2;
    public SubmergedVolumeMesh(Mesh mesh, Transform transform, SimulationScene scene) : base(mesh, transform, scene)
    {
    }
    public void CalculateSubmergedTriangleContribution(Vector3 a, Vector3 b, Vector3 c, Vector3 o, out float V, out Vector3 C)
    {
        // Signed volume of (abco) tetrahedra
        V = sign * Vector3.Dot(Vector3.Cross(a - o, b - o), c - o) / 6f;
        // Centroid of (abco) tetrahedra (multiplied by signed volume)
        C = (V * (a + b + c + o) / 4f);
    }
    protected override void CalculateCharacteristics()
    {
        tetrahedronVolumeElements = new float[triangleCount];
        untransformedTetrahedronCentroidElements = new Vector3[triangleCount];
        transformedTetrahedronCentroidElements = new Vector3[triangleCount];

        volume = 0;
        transformedCentroid = Vector3.zero;

        o = transform.TransformPoint(Vector3.zero);
        for (int i = 0; i < triangleCount; i++)
        {
            GetTriangleVertices(i, out a, out b, out c);
            CalculateSubmergedTriangleContribution(a, b, c, o, out tetrahedronVolumeElements[i], out transformedTetrahedronCentroidElements[i]);
            volume += tetrahedronVolumeElements[i];
            transformedCentroid += transformedTetrahedronCentroidElements[i];
        }

        if (volume < 0)
        {
            Debug.LogWarning("Negative volume detected. Trying to change signs of volume elements.");
            sign = -1;
            CalculateCharacteristics();
        }
        if (volume >= EPSILON)
        {
            transformedCentroid /= volume;
        }
        else
        {
            Debug.LogError("Unable to calculate submerged part. Volume cannot be zero.");
        }
        untransformedCentroid = transform.InverseTransformPoint(transformedCentroid);
        transform.InverseTransformPoints(transformedTetrahedronCentroidElements, untransformedTetrahedronCentroidElements);
    }

    protected override void UpdateVertexPositions()
    {
        base.UpdateVertexPositions();
        transform.TransformPoints(untransformedTetrahedronCentroidElements, transformedTetrahedronCentroidElements);
    }
    public void CalculatePartiallySubmergedTriangleContribution(int i, out float V, out Vector3 C)
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

    public float Volume
    {
        get => volume;
    }

    public float GetVolumeContributionOfTriangle(int triangleIndex)
    {
        return tetrahedronVolumeElements[triangleIndex];
    }
    public Vector3 GetTetrahedronCentroidOfTriangle(int triangleIndex)
    {
        return transformedTetrahedronCentroidElements[triangleIndex];
    }
}

public class SubmergedAreaMesh : SubmergedMesh
{
    protected Vector3[] untransformedTriangleCentroidElements;
    protected Vector3[] transformedTriangleCentroidElements;
    // The normals are not normalized. They have the same magnitude as the triangle's area.
    protected Vector3[] rotatedTriangleNormalElements;
    protected Vector3[] unrotatedTriangleNormalElements;
    public SubmergedAreaMesh(Mesh mesh, Transform transform, SimulationScene scene) : base(mesh, transform, scene)
    {
    }

    protected void CalculateSurfaceTriangleCharacteristics(Vector3 a, Vector3 b, Vector3 c, out Vector3 centroid, out Vector3 normal)
    {
        normal = Vector3.Cross(b - a, c - a) / 2;
        centroid = (a + b + c) / 3;
    }

    protected override void CalculateCharacteristics()
    {
        untransformedTriangleCentroidElements = new Vector3[triangleCount];
        transformedTriangleCentroidElements = new Vector3[triangleCount];
        unrotatedTriangleNormalElements = new Vector3[triangleCount];
        rotatedTriangleNormalElements = new Vector3[triangleCount];

        transformedCentroid = Vector3.zero;
        float volume = 0;

        for (int i = 0; i < triangleCount; i++)
        {
            GetTriangleVertices(i, out a, out b, out c);
            CalculateSurfaceTriangleCharacteristics(a, b, c, out transformedTriangleCentroidElements[i], out rotatedTriangleNormalElements[i]);
            // Using Divergence Theorem we can express the volume and the centroid parts without dealing with the volume
            // This is a discretization of the integrals obtained by the theorem
            volume += Vector3.Dot(transformedTriangleCentroidElements[i], rotatedTriangleNormalElements[i]) / 3f;
            transformedCentroid += Vector3.Scale(Vector3.Scale(transformedTriangleCentroidElements[i], transformedTriangleCentroidElements[i]) / 2f, rotatedTriangleNormalElements[i]);
        }

        if (volume >= EPSILON)
        {
            transformedCentroid /= volume;
        }
        else
        {
            Debug.LogError("Unable to calculate submerged part. Volume cannot be zero.");
        }
        untransformedCentroid = transform.InverseTransformPoint(transformedCentroid);
        transform.InverseTransformPoints(transformedTriangleCentroidElements, untransformedTriangleCentroidElements);
        transform.InverseTransformDirections(rotatedTriangleNormalElements, unrotatedTriangleNormalElements);
    }
    protected override void UpdateVertexPositions()
    {
        base.UpdateVertexPositions();
        transform.TransformPoints(untransformedTriangleCentroidElements, transformedTriangleCentroidElements);
        transform.TransformDirections(unrotatedTriangleNormalElements, rotatedTriangleNormalElements);
    }

    public Vector3 GetNormal(int triangleIdx)
    {
        return rotatedTriangleNormalElements[triangleIdx];
    }
    public Vector3 GetCentroid(int triangleIdx)
    {
        return transformedTriangleCentroidElements[triangleIdx];
    }
}
