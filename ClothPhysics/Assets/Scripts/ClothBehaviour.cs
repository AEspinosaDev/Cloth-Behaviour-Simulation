using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;


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

    #endregion 


    #region InEditorVariables

    [SerializeField] public Solver m_SolvingMethod;

    [SerializeField] [Range(0.001f, 0.02f)] private float m_TimeStep;

    [SerializeField] [Range(1, 20)] private int m_Substeps;

    [SerializeField] public bool m_Paused;

    //------Managed by custom editor class-----//

    public Ref<Vector3> m_Gravity = new Ref<Vector3>();

    public Ref<float> m_NodeMass = new Ref<float>();

    public Ref<float> m_NodeDamping = new Ref<float>();

    public Ref<float> m_SpringDamping = new Ref<float>();

    public Ref<float> m_TractionStiffness = new Ref<float>();

    public Ref<float> m_FlexionStiffness = new Ref<float>();

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

    [HideInInspector] public List<GameObject> m_CollidingObjects;

    #endregion

    public ClothBehaviour()
    {
        m_FixingByTexture = false;

        m_TimeStep = 0.004f;
        m_Substeps = 5;

        m_Gravity.value = new Vector3(0.0f, -9.81f, 0.0f);

        m_Paused = true;

        m_TractionStiffness.value = 20f;
        m_FlexionStiffness.value = 15f;

        m_NodeMass.value = 0.03f;

        m_NodeDamping.value = 0.3f;
        m_SpringDamping.value = 0.3f;

        m_SolvingMethod = Solver.Symplectic;

        m_FixedNodes = new List<Node>();

        m_AverageWindVelocity = Vector3.zero;

        m_AffectedByWind = false;
        m_WindFriction = 0.5f;
        m_WindSolverPrecission = WindPrecission.High;

        m_CanCollide = false;
    }

    #region Initialization Setups
    [ContextMenu("Low Res Mesh Setup")]
    private void LowResSetup()
    {
        m_TimeStep = 0.02f; m_Paused = true;
        m_TractionStiffness.value = 20f; m_FlexionStiffness.value = 15f; m_NodeMass.value = 0.03f; m_NodeDamping.value = 0.3f; m_SpringDamping.value = 0.3f;
        m_SolvingMethod = Solver.Symplectic;
        m_WindSolverPrecission = WindPrecission.High;
    }   
    [ContextMenu("Medium Res Mesh Setup")]
    private void MedResSetup()
    {
        m_TimeStep = 0.01f; m_Substeps = 2; m_Paused = true;
        m_TractionStiffness.value = 50f; m_FlexionStiffness.value = 30f; m_NodeMass.value = 0.03f; m_NodeDamping.value = 0.3f; m_SpringDamping.value = 0.3f;
        m_SolvingMethod = Solver.Symplectic;
        m_WindSolverPrecission = WindPrecission.Medium;
    }
    [ContextMenu("High Res Mesh Setup")]
    private void HighResSetup()
    {
        m_TimeStep = 0.007f; m_Substeps = 1; m_Paused = true;
        m_TractionStiffness.value = 100f; m_FlexionStiffness.value = 80f; m_NodeMass.value = 0.03f; m_NodeDamping.value = 0.3f; m_SpringDamping.value = 0.3f;
        m_SolvingMethod = Solver.Symplectic;
        m_WindSolverPrecission = WindPrecission.Low;
    }
    #endregion

    public enum Solver
    {
        Explicit = 0,
        Symplectic = 1,
        Midpoint = 2,
        Implicit = 3,
    };
    public enum WindPrecission
    {
        High = 1,
        Medium = 2,
        Low = 3,
    }


    #region MonoBehaviour

    public void Start()
    {
        m_Mesh = GetComponent<MeshFilter>().mesh;
        m_Vertices = m_Mesh.vertices;
        m_Triangles = m_Mesh.triangles;

        Vector2[] vertex_UVs = m_Mesh.uv;

        m_Nodes = new List<Node>();



        for (int i = 0; i < m_Mesh.vertexCount; i++)
        {
            //For simulation purposes, transform the points to global coordinates

            m_Nodes.Add(new Node(i, transform.TransformPoint(m_Vertices[i]), m_NodeMass, m_NodeDamping, m_Gravity, vertex_UVs[i]));
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

                    //edgeDictionary.Add(otherEdge, otherEdge);
                    m_Springs.Add(new Spring(m_Nodes[otherEdge.m_A], m_Nodes[otherEdge.m_B], m_FlexionStiffness, m_SpringDamping));

                }
                else
                {

                    //La arista no está en el diccionario
                    m_Springs.Add(new Spring(m_Nodes[edge.m_A], m_Nodes[edge.m_B], m_TractionStiffness, m_SpringDamping));
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

                case Solver.Explicit: stepExplicit(); break;

                case Solver.Symplectic: stepSymplectic(); break;

                case Solver.Midpoint: stepRK2(); break;

                //case Solver.Implicit: stepImplicit(); break;

                default:
                    throw new System.Exception("[ERROR] Should never happen!");

            }
        }

    }

    #endregion
    #region PhysicsSolvers
    /// <summary>
    /// Worst solver. Good for simple assets or arcade physics
    /// </summary>
    private void stepExplicit()
    {
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;
            //if (m_CanCollide && m_CollidingObjects.Count != 0) n.ComputePenaltyForce();
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
                n.m_Pos += m_SubTimeStep * n.m_Vel;
                n.m_Vel += m_SubTimeStep / n.m_Mass.value * n.m_Force;
            }
        }

    }
    /// <summary>
    /// Better solver. Either not perfect. Recommended.
    /// </summary>
    private void stepSymplectic()
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
                Vector3 vel = n.m_Vel + m_SubTimeStep / n.m_Mass.value * n.m_Force;

                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingObjects)
                    {

                        Collider collider = obj.GetComponent<Collider>();
                        if (collider.bounds.Contains(n.m_Pos))
                        {
                            MatrixXD df_dx = n.ComputePenaltyDifferential(collider);

                            //Vector3 closest = collider.ClosestPoint(n.m_Pos);
                            //Vector3 u = closest - n.m_Pos;
                            //Vector3 oX = u;
                            //float deepness = u.magnitude;
                            //u.Normalize();
                            //MatrixXD oxp= new DenseMatrixXD(1, 3);
                            //oxp[0, 0] = oX[0];
                            //oxp[0, 1] = oX[1];
                            //oxp[0, 2] = oX[2];
                            //oxp.Transpose();
                            

                            //VectorXD penaltyForce = -50f * deepness * u;

                            MatrixXD i = new DenseMatrixXD(3);
                            i = DenseMatrixXD.CreateIdentity(3);

                            VectorXD velProxy = new DenseVectorXD(3);
                            velProxy[0] = vel[0]; velProxy[1] = vel[1]; velProxy[2] = vel[2];
                            //MatrixXD velProxy = new DenseMatrixXD(1, 3);
                            //velProxy[0, 0] = vel[0];
                            //velProxy[0, 1] = vel[1];
                            //velProxy[0, 2] = vel[2];

                            //VectorXD resultProxy = (i - ((m_SubTimeStep * m_SubTimeStep) / n.m_Mass.value) * df_dx);

                            var resultProxy = (i - ((m_SubTimeStep * m_SubTimeStep) / n.m_Mass.value) * df_dx*-1f).Solve(velProxy);

                            vel = new Vector3((float)resultProxy[0], (float)resultProxy[1], (float)resultProxy[2]);
                            Debug.Log(vel);
                            break;

                        }
                    }
                }

                n.m_Vel = vel;
                n.m_Pos += m_SubTimeStep * n.m_Vel;
            }
        }

    }
    /// <summary>
    /// Fairly good solver. Also known as midpoint.
    /// </summary>
    private void stepRK2()
    {
        Vector3[] m_Vel0 = new Vector3[m_Nodes.Count];
        Vector3[] m_Pos0 = new Vector3[m_Nodes.Count];

        //Midpoint
        for (int i = 0; i < m_Nodes.Count; i++)
        {

            m_Vel0[i] = m_Nodes[i].m_Vel;
            m_Pos0[i] = m_Nodes[i].m_Pos;


            m_Nodes[i].m_Force = Vector3.zero;

            //if (m_CanCollide && m_CollidingObjects.Count != 0) m_Nodes[i].ComputePenaltyForce();
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
                n.m_Vel += (m_SubTimeStep * 0.5f) / n.m_Mass.value * n.m_Force;
                n.m_Pos += (m_SubTimeStep * 0.5f) * n.m_Vel;
            }
        }

        //EndPoint
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;

            //if (m_CanCollide && m_CollidingObjects.Count != 0) n.ComputePenaltyForce();
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
                m_Nodes[i].m_Vel = m_Vel0[i] + m_SubTimeStep / m_Nodes[i].m_Mass.value * m_Nodes[i].m_Force;
                m_Nodes[i].m_Pos = m_Pos0[i] + m_SubTimeStep * m_Nodes[i].m_Vel;
            }
        }


    }

    #endregion
    /// <summary>
    /// Checks whether the vertex is inside the fixers colliders in order to put it in fixer state
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
                    if (node.m_Fixer != null) Debug.LogWarning("More than one fixer assinged to the vertex. It will only be accepted one");
                    node.m_Fixer = obj;
                    node.m_offset = node.m_Fixer.transform.InverseTransformPoint(node.m_Pos);
                    m_FixedNodes.Add(node);

                }
                #region OwnCollisionAlgorithms
                //PROGRAMMED COLLISION ALGORITHMS

                //Vector3 f_center = collider.bounds.center;
                //    if (collider.GetType() == typeof(SphereCollider))
                //    {
                //        //Simple collision function with spheres
                //        float f_radius = obj.GetComponent<SphereCollider>().radius;
                //        float dist = (n_pos - f_center).magnitude;

                //        if (dist <= f_radius)
                //        {
                //            node.m_Fixed = true;
                //            if (node.m_Fixer != null) Debug.LogWarning("More than one fixer assinged to the vertex. It will only be accepted one");
                //            node.m_Fixer = obj;
                //            node.m_offset = node.m_Pos - node.m_Fixer.transform.position;
                //            m_FixedNodes.Add(node);
                //        }

                //    }
                //    else
                //    {
                //        //Simple collision function with boxes
                //        Vector3 f_size = collider.bounds.extents;
                //        //f_size = transform.InverseTransformPoint(f_size);

                //        float x_dist = Mathf.Abs(f_center.x - n_pos.x);
                //        float y_dist = Mathf.Abs(f_center.y - n_pos.y);
                //        float z_dist = Mathf.Abs(f_center.z - n_pos.z);


                //        int condition = 0;
                //        if (x_dist <= f_size.x) condition++;
                //        if (y_dist <= f_size.y) condition++;
                //        if (z_dist <= f_size.z) condition++;

                //        if (condition == 3)
                //        {
                //            node.m_Fixed = true;
                //            if (node.m_Fixer != null) Debug.LogWarning("More than one fixer assinged to the vertex. It will only be accepted one");
                //            node.m_Fixer = obj;
                //            node.m_offset = node.m_Pos - node.m_Fixer.transform.position;
                //            m_FixedNodes.Add(node);
                //        }

                //    }
                #endregion
            }
        }
    }
    private void CheckTextureWeights()
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
    /// <summary>
    /// Automatically called on start. Checks for wind objects in order to take them into account to make the wind simulation
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
                    Vector3 windVel = obj.transform.forward * (obj.windMain + (obj.windPulseMagnitude * obj.windMain * Mathf.Abs(Mathf.Sin(Time.time * (obj.windPulseFrequency * 10))))); //tweaked
                    m_AverageWindVelocity += windVel;
                }

            }
            if (m_AverageWindVelocity.magnitude > 0)
                m_AverageWindVelocity = new Vector3(m_AverageWindVelocity.x / total, m_AverageWindVelocity.y / total, m_AverageWindVelocity.z / total);
        }
    }
    /// <summary>
    /// Called every step update in the fixed update. Computes the applied wind force of every triangle in the mesh. 
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
    public Ref<float> m_Mass;
    public Ref<float> m_DampingA;
    public Ref<Vector3> m_Gravity;

    public Vector3 m_offset;            //Only if fixed enabled

    public GameObject m_Fixer;          //Only if fixed enabled

    public Vector3 m_WindForce;         //Only if wind enabled
    public Vector3 m_PenaltyForce;      //Only if collisions enabled

    public Vector2 m_UV;


    public Node(int iD, Vector3 pos, Ref<float> mass, Ref<float> damping, Ref<Vector3> grav, Vector2 uv)
    {
        m_Fixed = false;
        m_Id = iD;
        m_Pos = pos;
        m_Vel = Vector3.zero;
        m_UV = uv;
        m_ForceFactor = 1f;
        m_Fixer = null;
        m_DampingA = damping;
        m_Mass = mass;
        m_Gravity = grav;
    }

    /// <summary>
    /// Computes the resultant force of the node without taking in count the spring contraction force
    /// </summary>
    /// 
    public void ComputeForces()
    {

        m_Force += m_Mass.value * m_Gravity.value - m_DampingA.value * m_Mass.value * m_Vel + m_WindForce;

        m_Force *= m_ForceFactor;

        //m_PenaltyForce = Vector3.zero;

    }
    public MatrixXD ComputePenaltyDifferential(Collider collider)
    {
        float k = 50f;

        Vector3 closest = collider.ClosestPoint(m_Pos);
        Vector3 u = closest - m_Pos;
        Vector3 oX = u;
        //float deepness = u.magnitude;
        u.Normalize();

        //Vector3 penaltyForce = -k * deepness * u;
        //m_PenaltyForce = penaltyForce;
        //Debug.Log("Force"+penaltyForce);
        MatrixXD normal = new DenseMatrixXD(1, 3);
        normal[0, 0] = u[0];
        normal[0, 1] = u[1];
        normal[0, 2] = u[2];
        //MatrixXD oXProxy = new DenseMatrixXD(1, 3);
        //oXProxy[0, 0] = oX[0];
        //oXProxy[0, 1] = oX[1];
        //oXProxy[0, 2] = oX[2];

        var normalT = normal.Transpose();

        var normalMatrix = normalT * normal;
        //normalMatrix *= -k;

        return normalMatrix;

        //VectorXD oXproxy = new DenseVectorXD(3);
        //oXproxy[0] = oX[0];
        //oXproxy[1] = oX[1];
        //oXproxy[2] = oX[2];
        //var X = normalMatrix.Solve(oXproxy);
        //X *= -k;

        //Vector3 res = new Vector3((float)X[0], (float)X[1], (float)X[2]);

        //m_PenaltyForce = res;
        //Debug.Log(res);
        //Debug.Break();

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
    public Ref<float> m_Stiffness;
    public Ref<float> m_Damping;



    public Spring(Node nodeA, Node nodeB, Ref<float> stiff, Ref<float> damping)
    {
        m_NodeA = nodeA;
        m_NodeB = nodeB;
        m_Length0 = (m_NodeA.m_Pos - m_NodeB.m_Pos).magnitude;
        m_Length = m_Length0;
        m_Stiffness = stiff; //boin oi oi oi oing
        m_Damping = damping;
    }

    /// <summary>
    /// Compute de direction of the force
    /// </summary>
    public void ComputeForces()
    {

        Vector3 u = m_NodeA.m_Pos - m_NodeB.m_Pos;
        m_Length = u.magnitude;
        u.Normalize();

        float dampForce = -m_Damping.value * Vector3.Dot(u, (m_NodeA.m_Vel - m_NodeB.m_Vel));
        float stress = -m_Stiffness.value * (m_Length - m_Length0) + dampForce;
        Vector3 force = stress * u;
        m_NodeA.m_Force += force;
        m_NodeB.m_Force -= force;


    }

}
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