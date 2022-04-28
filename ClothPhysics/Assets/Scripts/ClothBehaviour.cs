using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;


///<author>
///Antonio Espinosa Garcia
///2022
///

/// <summary>
/// Cloth physics manager. Add this to a Scene Object with a mesh and let it flow with the wind
/// </summary>

public class ClothBehaviour : MonoBehaviour
{
    #region OtherVariables

    [HideInInspector] Mesh m_Mesh;

    [HideInInspector] private Vector3[] m_Vertices;

    [HideInInspector] private int[] m_Triangles;

    [HideInInspector] private List<Node> m_Nodes;

    [HideInInspector] private List<Spring> m_Springs;

    [HideInInspector] private List<Node> m_FixedNodes;

    [HideInInspector] private WindZone[] m_WindObjs;

    [HideInInspector] private Vector3 m_AverageWindVelocity;

    [HideInInspector] private float m_SubTimeStep;

    [HideInInspector] public float m_NodeMass;


    #endregion 


    #region InEditorVariables

    [Tooltip("The difference between the solvers lays on the precission they have calculating each vertex future position.")]
    [SerializeField] public Solver m_SolvingMethod;

    [Tooltip("Less time means more precission. On high res meshes is recommended to lower the timestep.")]
    [SerializeField] [Range(0.001f, 0.02f)] private float m_TimeStep;

    [Tooltip("It divides the total timestep into the number of substeps. More means more precission, but higher computational load.")]
    [SerializeField] [Range(1, 20)] private int m_Substeps;

    [Tooltip("Press P to pause/resume.")]
    [SerializeField] public bool m_Paused;

    [SerializeField] public Vector3 m_Gravity;

    [Tooltip("Controls the mass of the entire mesh, assuming it will be equally divided into each node.")]
    [SerializeField] [Range(0, 50)] public float m_MeshMass;

    [Tooltip("Higher values means more reduction in vertex movement.")]
    [SerializeField] [Range(0f, 5f)] public float m_NodeDamping;

    [Tooltip("Higher values means more reduction in spring contraction forces.")]
    [SerializeField] [Range(0f, 5f)] public float m_SpringDamping;

    [Tooltip("Controls the stiffness of the traction springs.These springs control horizontal and vertical movement. The more stiff, the less the mesh will deform and the quicker it will return to its initial state.")]
    [SerializeField] public float m_TractionStiffness;

    [Tooltip("Controls the stiffness of the flexion springs.These springs control shearing and diagonal movement.The more stiff, the less the mesh will deform and the quicker it will return to initial state.")]
    [SerializeField] public float m_FlexionStiffness;

    //------Managed by custom editor class-----//

    [HideInInspector] public bool m_AffectedByWind;

    [HideInInspector] public bool m_FixingByTexture;

    [HideInInspector] public bool m_CanCollide;
    #endregion

    #region ConditionalInEditorVariables
    //------Toggled by custom editor class only if enabled-----//

    [HideInInspector] [Range(0, 1)] public float m_WindFriction;
    [HideInInspector] public WindPrecission m_WindSolverPrecission;

    [HideInInspector] public Texture2D m_Texture;
    [HideInInspector] public List<GameObject> m_Fixers;

    [HideInInspector] public List<GameObject> m_CollidingMeshes;

    [HideInInspector] public float m_PenaltyStiffness;
    [HideInInspector] public float m_CollisionOffsetDistance;
    #endregion

    public ClothBehaviour()
    {
        m_FixingByTexture = false;

        m_TimeStep = 0.004f;
        m_Substeps = 5;

        m_Gravity = new Vector3(0.0f, -9.81f, 0.0f);

        m_Paused = true;

        m_TractionStiffness = 20f;
        m_FlexionStiffness = 15f;

        m_MeshMass = 3.63f;

        m_NodeDamping = 0.3f;
        m_SpringDamping = 0.3f;

        m_SolvingMethod = Solver.Simplectic;

        m_FixedNodes = new List<Node>();

        m_AverageWindVelocity = Vector3.zero;

        m_AffectedByWind = false;
        m_WindFriction = 0.5f;
        m_WindSolverPrecission = WindPrecission.High;

        m_CanCollide = false;
        m_PenaltyStiffness = 10f;
        m_CollisionOffsetDistance = 0.3f;
    }
    public enum Solver
    {
        Explicit = 0,
        Simplectic = 1,
        Midpoint = 2,
        SimplecticWithImplicitCollisions = 3,
    };
    public enum WindPrecission
    {
        High = 1,
        Medium = 2,
        Low = 3,
    }

    #region Initialization Setups
    [ContextMenu("Low Res Mesh Setup")]
    private void LowResSetup()
    {
        m_TimeStep = 0.02f; m_Paused = true;
        m_TractionStiffness = 20f; m_FlexionStiffness = 15f; m_MeshMass = 3.6f; m_NodeDamping = 0.3f; m_SpringDamping = 0.3f;
        m_SolvingMethod = Solver.Simplectic;
        m_WindSolverPrecission = WindPrecission.High;
    }
    [ContextMenu("Medium Res Mesh Setup")]
    private void MedResSetup()
    {
        m_TimeStep = 0.01f; m_Substeps = 2; m_Paused = true;
        m_TractionStiffness = 50f; m_FlexionStiffness = 30f; m_MeshMass = 20f; m_NodeDamping = 0.3f; m_SpringDamping = 0.3f;
        m_SolvingMethod = Solver.Simplectic;
        m_WindSolverPrecission = WindPrecission.Medium;
    }
    [ContextMenu("High Res Mesh Setup")]
    private void HighResSetup()
    {
        m_TimeStep = 0.007f; m_Substeps = 1; m_Paused = true;
        m_TractionStiffness = 100f; m_FlexionStiffness = 80f; m_MeshMass = 50f; m_NodeDamping = 0.3f; m_SpringDamping = 0.3f;
        m_SolvingMethod = Solver.Simplectic;
        m_WindSolverPrecission = WindPrecission.Low;
    }
    #endregion

    #region MonoBehaviour

    public void Start()
    {
        m_Mesh = GetComponent<MeshFilter>().mesh;
        m_Vertices = m_Mesh.vertices;
        m_Triangles = m_Mesh.triangles;

        Vector2[] vertex_UVs = m_Mesh.uv;

        m_Nodes = new List<Node>();

        m_NodeMass = m_MeshMass / m_Vertices.Length;

        for (int i = 0; i < m_Mesh.vertexCount; i++)
        {
            //For simulation purposes, transform the points to global coordinates

            m_Nodes.Add(new Node(i, transform.TransformPoint(m_Vertices[i]), this, vertex_UVs[i]));
        }


        m_Springs = new List<Spring>();

        EdgeQualityComparer edgeComparer = new EdgeQualityComparer();

        Dictionary<Edge, Edge> edgeDictionary = new Dictionary<Edge, Edge>(edgeComparer);

        for (int i = 0; i < m_Mesh.triangles.Length; i += 3)
        {

            List<Edge> edges = new List<Edge>();
            edges.Add(new Edge(m_Triangles[i], m_Triangles[i + 1], m_Triangles[i + 2]));
            edges.Add(new Edge(m_Triangles[i + 1], m_Triangles[i + 2], m_Triangles[i]));
            edges.Add(new Edge(m_Triangles[i], m_Triangles[i + 2], m_Triangles[i + 1]));
            foreach (var edge in edges)
            {
                Edge otherEdge;
                if (edgeDictionary.TryGetValue(edge, out otherEdge))
                {
                    //La arista está en el diccionario
                    otherEdge.m_A = otherEdge.m_O;
                    otherEdge.m_B = edge.m_O;
                    m_Springs.Add(new Spring(m_Nodes[otherEdge.m_A], m_Nodes[otherEdge.m_B], this, false));

                }
                else
                {

                    //La arista no está en el diccionario
                    m_Springs.Add(new Spring(m_Nodes[edge.m_A], m_Nodes[edge.m_B], this, true));
                    edgeDictionary.Add(edge, edge);
                }
            }
        }

        m_SubTimeStep = m_TimeStep / m_Substeps;

        //Attach to fixers
        if (m_FixingByTexture)
            CheckTextureWeights();
        else
            CheckFixers();

        //Look for Wind objs
        CheckWindObjects();
    }

    public void Update()
    {
        if (Input.GetKeyUp(KeyCode.P))
            this.m_Paused = !this.m_Paused;

        m_NodeMass = m_MeshMass / m_Vertices.Length;
        m_SubTimeStep = m_TimeStep / m_Substeps;

        CheckWindObjects();
        if (m_AffectedByWind && m_WindSolverPrecission == WindPrecission.Low)
        {
            ComputeWindForces();
        }

        foreach (var node in m_FixedNodes)
        {
            if (m_FixingByTexture)
            {
                node.m_Pos = transform.TransformPoint(node.m_offset);
            }
            else
            {
                node.m_Pos = node.m_Fixer.transform.TransformPoint(node.m_offset);

            }
        }


        for (int i = 0; i < m_Mesh.vertexCount; i++)
        {
            m_Vertices[i] = transform.InverseTransformPoint(m_Nodes[i].m_Pos);
        }

        m_Mesh.vertices = m_Vertices;

        m_Mesh.RecalculateNormals();
    }

    public void FixedUpdate()
    {
        if (m_Paused)
            return; // Not simulating

        if (m_AffectedByWind && m_WindSolverPrecission == WindPrecission.Medium)
        {
            ComputeWindForces();
        }

        // Select integration method
        for (int i = 0; i < m_Substeps; i++)
        {
            if (m_AffectedByWind && m_WindSolverPrecission == WindPrecission.High)
            {
                ComputeWindForces();
            }

            switch (m_SolvingMethod)
            {

                case Solver.Explicit: StepExplicit(); break;

                case Solver.Simplectic: StepSimplectic(); break;

                case Solver.Midpoint: StepRK2(); break;

                case Solver.SimplecticWithImplicitCollisions: StepSimplecticWithImplicitCollision(); break;

                default:
                    throw new System.Exception("[ERROR] Should never happen!");

            }
        }

    }

    #endregion

    #region PhysicsSolvers
    /// <summary>
    /// Worst solver. Good for simple assets or arcade physics.
    /// </summary>
    private void StepExplicit()
    {
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;
            if (!m_AffectedByWind) n.m_WindForce = Vector3.zero;
            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = n.isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            n.ComputeExplicitPenaltyForce(n.ComputeCollision(obj, condition), m_PenaltyStiffness);
                            n.m_Force += n.m_PenaltyForce;
                        }
                    }
                }
                n.m_Pos += m_SubTimeStep * n.m_Vel;
                n.m_Vel += m_SubTimeStep / m_NodeMass * n.m_Force;
            }
        }

    }
    /// <summary>
    /// Better solver. Either not perfect. Recommended.
    /// </summary>
    private void StepSimplectic()
    {

        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;
            if (!m_AffectedByWind) n.m_WindForce = Vector3.zero;
            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                Vector3 resVel = n.m_Vel + m_SubTimeStep / m_NodeMass * n.m_Force;
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = n.isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            n.ComputeExplicitPenaltyForce(n.ComputeCollision(obj, condition), m_PenaltyStiffness);
                            n.m_Force += n.m_PenaltyForce;
                            resVel = n.m_Vel + m_SubTimeStep / m_NodeMass * n.m_Force;
                        }
                    }
                }
                n.m_Vel = resVel;
                n.m_Pos += m_SubTimeStep * n.m_Vel;
            }

        }
    }

    /// <summary>
    /// Fairly good solver. Also known as midpoint.
    /// </summary>
    private void StepRK2()
    {
        Vector3[] m_Vel0 = new Vector3[m_Nodes.Count];
        Vector3[] m_Pos0 = new Vector3[m_Nodes.Count];

        //Midpoint
        for (int i = 0; i < m_Nodes.Count; i++)
        {

            m_Vel0[i] = m_Nodes[i].m_Vel;
            m_Pos0[i] = m_Nodes[i].m_Pos;


            m_Nodes[i].m_Force = Vector3.zero;

            if (!m_AffectedByWind) m_Nodes[i].m_WindForce = Vector3.zero;
            m_Nodes[i].ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                Vector3 resVel = n.m_Vel + (m_SubTimeStep * 0.5f) / m_NodeMass * n.m_Force;
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = n.isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            n.ComputeExplicitPenaltyForce(n.ComputeCollision(obj, condition), m_PenaltyStiffness);
                            n.m_Force += n.m_PenaltyForce;
                            resVel = n.m_Vel + (m_SubTimeStep * 0.5f) / m_NodeMass * n.m_Force;
                        }
                    }
                }
                n.m_Vel += resVel;
                n.m_Pos += (m_SubTimeStep * 0.5f) * n.m_Vel;
            }
        }

        //EndPoint
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;

            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        for (int i = 0; i < m_Nodes.Count; i++)
        {

            if (!m_Nodes[i].m_Fixed)
            {
                Vector3 resVel = m_Vel0[i] + m_SubTimeStep / m_NodeMass * m_Nodes[i].m_Force;
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = m_Nodes[i].isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            m_Nodes[i].ComputeExplicitPenaltyForce(m_Nodes[i].ComputeCollision(obj, condition), m_PenaltyStiffness);
                            m_Nodes[i].m_Force += m_Nodes[i].m_PenaltyForce;
                            resVel = m_Vel0[i] + m_SubTimeStep / m_NodeMass * m_Nodes[i].m_Force;
                        }
                    }
                }
                m_Nodes[i].m_Vel = resVel;
                m_Nodes[i].m_Pos = m_Pos0[i] + m_SubTimeStep * m_Nodes[i].m_Vel;
            }
        }


    }
    /// <summary>
    /// Simplectic solver using implicit aproximation for collisions. The best solver.
    /// </summary>
    private void StepSimplecticWithImplicitCollision()
    {
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;
            if (!m_AffectedByWind) n.m_WindForce = Vector3.zero;
            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        Vector3 resVel;
        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                resVel = n.m_Vel + m_SubTimeStep / m_NodeMass * n.m_Force;
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = n.isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            MatrixXD diff = n.ComputeImplicitPenaltyForce(n.ComputeCollision(obj, condition), m_PenaltyStiffness);

                            MatrixXD i = new DenseMatrixXD(3);
                            i = DenseMatrixXD.CreateIdentity(3);

                            Vector3 b = n.m_Vel + m_SubTimeStep / m_NodeMass * (n.m_PenaltyForce + n.m_Force); //Spring and wind force already computed in n.m_Force

                            VectorXD bProxy = new DenseVectorXD(3);
                            bProxy[0] = b.x; bProxy[1] = b.y; bProxy[2] = b.z;

                            var x = (i - (m_SubTimeStep * m_SubTimeStep / m_NodeMass) * diff).Solve(bProxy);

                            resVel = new Vector3((float)x[0], (float)x[1], (float)x[2]);

                        }
                    }

                }

                n.m_Vel = resVel;
                n.m_Pos += m_SubTimeStep * n.m_Vel;
            }

        }
    }


    #endregion
    /// <summary>
    /// Checks whether the vertex is inside the fixer colliders in order to put it in a fixed state.
    /// </summary>
    private void CheckFixers()
    {
        foreach (var node in m_Nodes)
        {
            foreach (var obj in m_Fixers)
            {
                Collider collider = obj.GetComponent<Collider>();
                Vector3 n_pos = node.m_Pos;

                if (collider.bounds.Contains(n_pos))
                {
                    node.m_Fixed = true;
                    if (node.m_Fixer != null) Debug.LogWarning("[Warning] More than one fixer assinged to the vertex. It will only be accepted one");
                    node.m_Fixer = obj;
                    node.m_offset = node.m_Fixer.transform.InverseTransformPoint(node.m_Pos);
                    m_FixedNodes.Add(node);

                }
            }
        }
    }
    /// <summary>
    /// Check whether the vertex color value is inside the fixing color condition in order to put it in a fixed state.
    /// </summary>
    private void CheckTextureWeights()
    {
        if (m_Texture != null)
        {
            int textWidth = m_Texture.width;

            foreach (var node in m_Nodes)
            {
                float xCoord = textWidth * node.m_UV.x;
                float yCoord = textWidth * node.m_UV.y;

                Color color = m_Texture.GetPixel((int)xCoord, (int)yCoord);

                float factor = 0.9f;
                if (color.a >= factor)
                {
                    node.m_Fixed = true;
                    m_FixedNodes.Add(node);
                    node.m_offset = transform.InverseTransformPoint(node.m_Pos);
                }
                else
                {
                    node.m_ForceFactor = 1 - color.a;
                }
            }
        }

    }
    /// <summary>
    /// Automatically called on start. Checks for wind objects in order to take them into account to make the wind simulation.
    /// </summary>
    private void CheckWindObjects()
    {
        if (m_AffectedByWind) m_WindObjs = FindObjectsOfType<WindZone>();
        ComputeWindSpeed();
    }
    /// <summary>
    /// Called every update. Computes the average velocity vector of the resulting wind.
    /// </summary>
    private void ComputeWindSpeed()
    {
        m_AverageWindVelocity = Vector3.zero;

        if (m_AffectedByWind)
        {
            int total = m_WindObjs.Length;
            foreach (var obj in m_WindObjs)
            {
                if (obj.gameObject.activeSelf)
                {
                    //Takes in account all wind object parameters to simulate wind variation
                    Vector3 windVel = obj.transform.forward * (obj.windMain + (obj.windPulseMagnitude * obj.windMain * Mathf.Abs(Mathf.Sin(Time.time * (obj.windPulseFrequency * 10)))));
                    m_AverageWindVelocity += windVel;
                }

            }
            if (m_AverageWindVelocity.magnitude > 0)
                m_AverageWindVelocity = new Vector3(m_AverageWindVelocity.x / total, m_AverageWindVelocity.y / total, m_AverageWindVelocity.z / total);
        }
    }
    /// <summary>
    /// Called every wind solving iteration. Computes the applied wind force of every triangle in the mesh. 
    /// </summary>
    private void ComputeWindForces()
    {
        for (int i = 0; i < m_Mesh.triangles.Length; i += 3)
        {

            Node nodeA = m_Nodes[m_Triangles[i]];
            Node nodeB = m_Nodes[m_Triangles[i + 1]];
            Node nodeC = m_Nodes[m_Triangles[i + 2]];

            Vector3 crossProduct = Vector3.Cross(nodeA.m_Pos - nodeB.m_Pos, nodeB.m_Pos - nodeC.m_Pos);

            float trisArea = crossProduct.magnitude / 2;

            Vector3 trisNormal = crossProduct.normalized;

            Vector3 trisSpeed = new Vector3((nodeA.m_Vel.x + nodeB.m_Vel.x + nodeC.m_Vel.x) / 3, (nodeA.m_Vel.y + nodeB.m_Vel.y + nodeC.m_Vel.y) / 3, (nodeA.m_Vel.z + nodeB.m_Vel.z + nodeC.m_Vel.z) / 3);

            Vector3 trisWindForce = m_WindFriction * trisArea * Vector3.Dot(trisNormal, m_AverageWindVelocity - trisSpeed) * trisNormal;
            Vector3 dispersedForce = trisWindForce / 3;

            nodeA.m_WindForce = dispersedForce;
            nodeB.m_WindForce = dispersedForce;
            nodeC.m_WindForce = dispersedForce;
        }
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//AUXILIAR CLASSES

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/// <summary>
/// Class representing each vertex physical attributes necessary
/// to compute the cloth physics
/// </summary>

public class Node
{
    public bool m_Fixed;

    public readonly int m_Id;
    public Vector3 m_Pos;
    public Vector3 m_Vel;
    public Vector3 m_Force;
    public float m_ForceFactor;


    public Vector3 m_offset;            //Only if fixed enabled

    public GameObject m_Fixer;          //Only if fixed enabled

    public Vector3 m_WindForce;         //Only if wind enabled
    public Vector3 m_PenaltyForce;      //Only if collisions enabled

    public Vector2 m_UV;
    public ClothBehaviour m_Manager;


    public Node(int iD, Vector3 pos, ClothBehaviour manager, Vector2 uv)
    {
        m_Fixed = false;
        m_Id = iD;
        m_Pos = pos;
        m_Vel = Vector3.zero;
        m_UV = uv;
        m_ForceFactor = 1f;
        m_Fixer = null;

        m_Manager = manager;

    }

    /// <summary>
    /// Computes the resultant force of the node without taking in count the spring contraction force
    /// </summary>
    /// 
    public void ComputeForces()
    {

        m_Force += m_Manager.m_NodeMass * m_Manager.m_Gravity - m_Manager.m_NodeDamping * m_Manager.m_NodeMass * m_Vel;

        m_Force += m_WindForce;

        m_Force *= m_ForceFactor;

        m_PenaltyForce = Vector3.zero;

    }
    /// <summary>
    /// Checks if the node is inside the collider of an object. Currently, it only supports thre types of colliders: spheric, plane and box.
    /// </summary>
    /// <param name="obj"></param>
    /// <returns>Returns the type of collision. 1 if its sphere type. -1 if its plane type. 0 if theres no collision</returns>
    public int isColliding(GameObject obj, float offset)
    {
        if (obj.GetComponent<SphereCollider>() != null)
        {
            SphereCollider collider = obj.GetComponent<SphereCollider>();

            if (collider.transform.InverseTransformPoint(m_Pos).magnitude < collider.radius + offset) return 1;

        }
        else if (obj.GetComponent<MeshCollider>() != null)
        {
            MeshCollider collider = obj.GetComponent<MeshCollider>();
            float xExtension = collider.sharedMesh.bounds.extents.x;
            float zExtension = collider.sharedMesh.bounds.extents.z;
            Vector3 nodeLocalPos = obj.transform.InverseTransformPoint(m_Pos);
            if (nodeLocalPos.y <= 0f + offset && nodeLocalPos.y >= -1.0f && Mathf.Abs(nodeLocalPos.z) <= zExtension && Mathf.Abs(nodeLocalPos.x) <= xExtension) return 2;
        }
        else
        {
            BoxCollider collider = obj.GetComponent<BoxCollider>();
            Vector3 offsetDir = (m_Pos - collider.transform.position).normalized;
            if (collider.bounds.Contains(m_Pos + (offsetDir * offset))) return 3;

        }
        return 0;

    }
    /// <summary>
    /// Computes the closest point in any kind of supported collider surface to the node position
    /// </summary>
    /// <param name="obj"></param>
    /// <param name="condition"></param>
    /// <returns>Returns the coordinates of the impact point</returns>
    public Vector3 ComputeCollision(GameObject obj, float condition)
    {
        Vector3 impactPoint;
        switch (condition)
        {
            case 1:
                impactPoint = ComputeSphereCollision(obj); break;
            case 2:
                impactPoint = ComputePlaneCollision(obj); break;
            case 3:
                impactPoint = ComputeBoxCollision(obj); break;
            default:
                throw new System.Exception("[ERROR] Should never happen!");
        }
        return impactPoint;
    }
    /// <summary>
    /// Computes the closest point in the sphere collider surface to the node position
    /// </summary>
    /// <param name="obj"></param>
    /// <returns>Returns the coordinates of the impact point</returns>
    private Vector3 ComputeSphereCollision(GameObject obj)
    {
        SphereCollider collider = obj.GetComponent<SphereCollider>();
        Vector3 impactPoint;
        Vector3 impactDir = m_Pos - collider.transform.position;
        impactPoint = m_Pos + Vector3.ClampMagnitude(impactDir, collider.radius * obj.transform.lossyScale.x);
        return impactPoint;

    }
    /// <summary>
    /// Computes the closest point in the plane collider surface to the node position
    /// </summary>
    /// <param name="obj"></param>
    /// <returns>Returns the coordinates of the impact point</returns>
    private Vector3 ComputePlaneCollision(GameObject obj)
    {
        MeshCollider collider = obj.GetComponent<MeshCollider>();
        Vector3 impactPoint = m_Pos + (collider.transform.up * (-collider.transform.InverseTransformPoint(m_Pos).y + m_Manager.m_CollisionOffsetDistance));
        return impactPoint;
    }
    /// <summary>
    /// Computes the closest point in the box collider surface to the node position
    /// </summary>
    /// <param name="obj"></param>
    /// <returns>Returns the coordinates of the impact point</returns>
    private Vector3 ComputeBoxCollision(GameObject obj)
    {
        BoxCollider collider = obj.GetComponent<BoxCollider>();
        Vector3 localNodePos = collider.transform.InverseTransformPoint(m_Pos).normalized;
        float angleX = Vector3.Angle(collider.transform.right, localNodePos);
        float angleY = Vector3.Angle(collider.transform.up, localNodePos);
        float angleZ = Vector3.Angle(collider.transform.forward, localNodePos);
        if (angleX < angleY && angleX < angleZ) return collider.transform.right;
        if (angleY < angleX && angleY < angleZ) return collider.transform.right;
        if (angleZ < angleY && angleZ < angleX) return collider.transform.right;
        throw new System.Exception("[ERROR] Should never happen!");

    }
    /// <summary>
    /// Implicitly computes the penalty force by calculating the penalty force and the penalty force differential matrix.
    /// </summary>
    /// <param name="impactPoint"></param>
    /// <param name="k"></param>
    /// <returns>Returns the differential</returns>
    public MatrixXD ComputeImplicitPenaltyForce(Vector3 impactPoint, float k)
    {

        Vector3 u = m_Pos - impactPoint;
        Vector3 oX = u;
        u.Normalize();
        MatrixXD normal = new DenseMatrixXD(3, 1);
        normal[0, 0] = u[0];
        normal[1, 0] = u[1];
        normal[2, 0] = u[2];
        MatrixXD oXProxy = new DenseMatrixXD(3, 1);
        oXProxy[0, 0] = oX[0];
        oXProxy[1, 0] = oX[1];
        oXProxy[2, 0] = oX[2];

        var normalT = normal.Transpose();

        var diff = -k * normal * normalT;

        MatrixXD penaltyForce = -k * normal * normalT * oXProxy;

        m_PenaltyForce[0] = (float)penaltyForce[0, 0];
        m_PenaltyForce[1] = (float)penaltyForce[1, 0];
        m_PenaltyForce[2] = (float)penaltyForce[2, 0];

        return diff;

    }
    /// <summary>
    /// Explicitly computes the penalty force on a point in time.
    /// </summary>
    /// <param name="impactPoint"></param>
    /// <param name="k"></param>
    public void ComputeExplicitPenaltyForce(Vector3 impactPoint, float k)
    {
        Vector3 normal = m_Pos - impactPoint;
        float deepness = normal.magnitude;

        normal.Normalize();

        m_PenaltyForce = -k * deepness * normal;

    }
}
/// <summary>
/// Class representing each edge interacting with nodes and aplying compression and traction forces
/// </summary>
public class Spring
{

    public Node m_NodeA, m_NodeB;

    public float m_Length0;
    public float m_Length;
    public float m_Stiffness;

    bool m_TypeTraction;
    ClothBehaviour m_Manager;

    public Spring(Node nodeA, Node nodeB, ClothBehaviour manager, bool type)
    {
        m_NodeA = nodeA;
        m_NodeB = nodeB;
        m_Length0 = (m_NodeA.m_Pos - m_NodeB.m_Pos).magnitude;
        m_Length = m_Length0;
        m_Manager = manager;
        m_TypeTraction = type;
    }

    /// <summary>
    /// Compute de direction of the force
    /// </summary>
    public void ComputeForces()
    {

        if (m_TypeTraction) m_Stiffness = m_Manager.m_TractionStiffness;
        else m_Stiffness = m_Manager.m_FlexionStiffness;

        Vector3 u = m_NodeA.m_Pos - m_NodeB.m_Pos;
        m_Length = u.magnitude;
        u.Normalize();

        float dampForce = -m_Manager.m_SpringDamping * Vector3.Dot(u, (m_NodeA.m_Vel - m_NodeB.m_Vel));
        float stress = -m_Stiffness * (m_Length - m_Length0) + dampForce;
        Vector3 force = stress * u;
        m_NodeA.m_Force += force;
        m_NodeB.m_Force -= force;

    }
}
/// <summary>
/// Auxiliar class to temporary compare the edges in the mesh looking for repeated ones.
/// </summary>
public class Edge
{
    public int m_A, m_B, m_O;

    public Edge(int a, int b, int o)
    {
        if (a < b)
        {
            m_A = a;
            m_B = b;

        }
        else
        {
            m_B = a;
            m_A = b;
        }
        m_O = o;
    }
}

public class EdgeQualityComparer : IEqualityComparer<Edge>
{
    public bool Equals(Edge a, Edge b)
    {
        if (a.m_A == b.m_A && a.m_B == b.m_B || a.m_A == b.m_B && a.m_B == b.m_A) return true;
        else return false;
    }

    public int GetHashCode(Edge e)
    {
        List<int> pts = new List<int>(); pts.Add(e.m_A); pts.Add(e.m_B);
        pts.Sort();
        //CANTOR PAIRING FUNCTION
        int hcode = ((pts[0] + pts[1]) * (pts[0] + pts[1] + 1)) / 2 + pts[1];

        return hcode.GetHashCode();
    }
}
public class Ref<T>
{
    public T value { get; set; }
}